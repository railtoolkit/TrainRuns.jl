#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

module TrainRunCalc

# include modules of TrainRunCalc
include("./Input.jl")
include("./Characteristics.jl")
include("./Behavior.jl")
include("./Output.jl")


# use modules of TrainRunCalc
using .Input
using .Characteristics
using .Behavior
using .Output

# export main function
export calculateDrivingDynamics

approximationLevel = 6  # value for approximation to intersections and precisely calculated digits
    # TODO:  define it here and give it to each function? (Behavior, ...)

# Calculate the driving dynamics of a train run on a path with special settings with information from the corresponding YAML files with the file paths `trainDirectory`, `pathDirectory`, `settingsDirectory`.

"""
    calculateDrivingDynamics(train::Dict, path::Dict, settings::Dict)

Calculate the driving dynamics of a train run on a path with special settings with information from the corresponding dictionaries `train`, `path`, `settings`.

# Examples
```julia-repl
julia> calculateDrivingDynamics(trainDict, pathDict, settingsDict)
todo !!!
```
"""
function calculateDrivingDynamics(trainInput::Dict, pathInput::Dict, settingsInput::Dict)
    # copy Input data for not changing them
    # TODO: or should they be changed? normally it would only make it "better" except for settings[:detailOfOutput] == "points of interest" && !haskey(path, :pointsOfInterest)
    train = copy(trainInput)
    path = copy(pathInput)
    settings = copy(settingsInput)

    # check the input data
    (train, path, settings) = checkAndSetInput!(train, path, settings)
    println("The input has been checked.")

    # prepare the input data
    movingSection = preparateSections(path, train, settings)
    println("The moving section has been prepared.")

    # calculate the train run for oparation mode "minimum running time"
    if settings[:operationModeMinimumRunningTime] || settings[:operationModeMinimumEnergyConsumption]
        (movingSection, drivingCourse) = calculateMinimumRunningTime!(movingSection, settings, train)
        println("The driving course for the shortest running time has been calculated.")

        # accumulate data and create an output dictionary
        output = createOutputDict(train, settings, path, movingSection, drivingCourse)
    else
        output = Dict()
    end #if

    return output
end # function calculateDrivingDynamics

# calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime!(movingSection::Dict, settings::Dict, train::Dict)
   CSs::Vector{Dict} = movingSection[:characteristicSections]

   if settings[:massModel] == "homogeneous strip"
       println("WARNING: ! ! ! TrainRun.jl doesn't work reliably for the mass model homogeneous strip. This mass model should not be used ! ! !")
   end

   startingPoint=createDataPoint()
   startingPoint[:i]=1
   startingPoint[:s]=CSs[1][:s_entry]
   calculateForces!(startingPoint, CSs, 1, "default", train, settings[:massModel]) # traction effort and resisting forces (in N)
   drivingCourse::Vector{Dict} = [startingPoint]    # List of data points

   for csId in 1:length(CSs)
       CS = CSs[csId]

           # for testing
           if drivingCourse[end][:s] != CS[:s_entry]
               println("ERROR: In CS", csId," the train run starts at s=",drivingCourse[end][:s]," and not s_entry=",CS[:s_entry])
           end
           if drivingCourse[end][:v] > CS[:v_entry]
               println("ERROR: In CS", csId," the train run ends with v=",drivingCourse[end][:v]," and not with v_entry=",CS[:v_entry])
           end

       # determine the different flags for switching between the states for creatinge moving phases
       s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
       calculateForces!(drivingCourse[end], CSs, CS[:id], "default", train, settings[:massModel])     # tractive effort and resisting forces (in N)

       previousSpeedLimitReached = false
       stateFlags = Dict(:endOfCSReached => drivingCourse[end][:s] > CS[:s_exit],
                         :brakingStartReached => drivingCourse[end][:s] + s_braking == CS[:s_exit],
                         :tractionDeficit => drivingCourse[end][:F_T] < drivingCourse[end][:F_R], # or add another flag for equal forces?
                         :previousSpeedLimitReached => false, #speedLimitReached, # check already at this position?
                         :speedLimitReached => drivingCourse[end][:v] > CS[:v_limit],
                         :error => false)
# TODO: add stateFlag :resistingForcesNegative for leaving cruising in favor of downhilBraking in homogenous strip

     # determine the behavior sections for this characteristic section. It has to be at least one of those BS: "breakFree", "clearing", "accelerating", "cruising", "diminishing", "coasting", "braking" or "standstill")
    while !stateFlags[:endOfCSReached] # s < s_exit
        if !stateFlags[:brakingStartReached] # s+s_braking < s_exit
            if !stateFlags[:tractionDeficit]
                if drivingCourse[end][:F_T] >  drivingCourse[end][:F_R] && drivingCourse[end][:v] == 0.0
                    (CS, drivingCourse, stateFlags) = addBreakFreeSection!(CS, drivingCourse, stateFlags, settings, train, CSs)

                #elseif stateFlags[:previousSpeedLimitReached]
                #    println("clear")

                elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R] && !stateFlags[:speedLimitReached] # v <  v_limit
                    (CS, drivingCourse, stateFlags) = addAcceleratingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)

                elseif  drivingCourse[end][:F_R] < 0 && stateFlags[:speedLimitReached] # v <  v_limit
                    if settings[:massModel] == "mass point"
                        s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                        s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking
                    elseif settings[:massModel] == "homogeneous strip"
                        # TODO: Add downhillBraking for homogeneous strip
                        error("Add downhillBraking for homogeneous strip !")
                        # cruise until F_R >= 0.0
                    end

                    if s_cruising > 0.0  # TODO: define a minimum cruising length?
                        (CS, drivingCourse, brakingStartReached) = addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "downhillBraking")
                        stateFlags[:brakingStartReached] = brakingStartReached
                    else
                        stateFlags[:brakingStartReached] = true
                    end

                elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R] || stateFlags[:speedLimitReached]
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                    s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                    if s_cruising > 0.0  # TODO: define a minimum cruising length?
                        (CS, drivingCourse, brakingStartReached) = addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "cruising")
                        stateFlags[:brakingStartReached] = brakingStartReached
                    else
                        stateFlags[:brakingStartReached] = true
                    end
                else
                    error()
                end
            elseif stateFlags[:tractionDeficit]
                (CS, drivingCourse, stateFlags) = addDiminishingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)

            else
                 error()
            end
        else#if !stateFlags[:endOfCSReached] # s < s_exit
            (CS, drivingCourse, stateFlags) = addBrakingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)
        #else
        #   error()
        end
    end
    #if s == s_exit
    #    standstill
    #end


#=
        # determine the behavior sections for this characteristic section. It has to be at least one of those BS: "breakFree", "clearing", "accelerating", "cruising", "diminishing", "coasting", "braking" or "standstill")

        if drivingCourse[end][:v] == 0.0
            (CS, drivingCourse, stateFlags) = addBreakFreeSection!(CS, drivingCourse, stateFlags, settings, train, CSs)
        end #if

        drivingCourse[end][:s] + s_braking > CS[:s_exit] && error("ERROR: In CS", csId,": s +s_braking=", drivingCourse[end][:s],",+",s_braking," > ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])
        testFlag = false     # for testing

        brakingStartReached = stateFlags[:brakingStartReached]
        while !brakingStartReached
            calculateForces!(drivingCourse[end], CSs, CS[:id], "default", train, settings[:massModel])     # traction effort and resisting forces (in N)

            if drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
                if drivingCourse[end][:v] < CS[:v_peak] - 1/10^approximationLevel * settings[:stepSize] # TODO: check if multiplying with stepSize is necessary
                    (CS, drivingCourse, stateFlags) = addAcceleratingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)
                    brakingStartReached = stateFlags[:brakingStartReached]
                    #    testFlag && println("in CS",CS[:id]," after accelerating s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," <= s_exit=",CS[:s_exit])    # for testing
                else
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                    s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                    if s_cruising > 0.0  # TODO: define a minimum cruising length?

                            (CS, drivingCourse, brakingStartReached) = addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "cruising")
                            #s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                            #    testFlag && println("in CS",CS[:id]," after cruising s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," <= s_exit=",CS[:s_exit])    # for testing

            #            TODO: add downhillBraking as a special cruising Section:
            #                (CS, drivingCourse, brakingStartReached) = addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "downhillBraking")

                    end
                end #if
            else
                (CS, drivingCourse, stateFlags) = addDiminishingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)
                brakingStartReached = stateFlags[:brakingStartReached]
                #s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                #    testFlag && println("in CS",CS[:id]," after diminishing s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," <= s_exit=",CS[:s_exit])    # for testing
            end

            s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
            if drivingCourse[end][:s] +s_braking == CS[:s_exit]
                brakingStartReached = true
            end
        end

        if drivingCourse[end][:v] > CS[:v_exit]
            (CS, drivingCourse, stateFlags) = addBrakingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)
        end #if

=#
           # for testing:
           if drivingCourse[end][:s] != CS[:s_exit]
               println("ERROR: In CS", csId," the train run ends at s=",drivingCourse[end][:s]," and not s_exit=",CS[:s_exit])
           end
           if drivingCourse[end][:v] > CS[:v_exit]
               println("ERROR: In CS", csId," the train run ends with v=",drivingCourse[end][:v]," and not with v_exit=",CS[:v_exit])
           end
   end #for

   (CSs[end], drivingCourse) = addStandstill!(CSs[end], drivingCourse, settings, train, CSs)

   movingSection[:t] = drivingCourse[end][:t]            # total running time (in s)
   movingSection[:E] = drivingCourse[end][:E]            # total energy consumption (in Ws)

   return (movingSection, drivingCourse)
end #function calculateMinimumRunningTime

end # module TrainRunCalc
