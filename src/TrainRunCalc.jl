#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

module TrainRunCalc

# include modules of TrainRunCalc
include("./Validate.jl")
include("./Characteristics.jl")
include("./Behavior.jl")
include("./Output.jl")


# use modules of TrainRunCalc
using .Validate
using .Characteristics
using .Behavior
using .Output

# export main function
export trainRun

approximationLevel = 6  # value for approximation to intersections and precisely calculated digits
    # TODO:  define it here and give it to each function? (Behavior, ...)

# Calculate the driving dynamics of a train run on a path with special settings with information from the corresponding YAML files with the file paths `trainDirectory`, `pathDirectory`, `settingsDirectory`.

"""
    trainRun(train::Dict, path::Dict, settings::Dict)

Calculate the driving dynamics of a train run on a path with special settings with information from the corresponding dictionaries `train`, `path`, `settings`.

# Examples
```julia-repl
julia> trainRun(trainDict, pathDict, settingsDict)
todo !!!
```
"""
function trainRun(trainInput::Dict, pathInput::Dict, settingsInput::Dict)
    # copy Input data for not changing them
    # TODO: or should they be changed? normally it would only make it "better" except for settings[:detailOfOutput] == "points of interest" && !haskey(path, :pointsOfInterest)
    train = copy(trainInput)
    path = copy(pathInput)
    settings = copy(settingsInput)

    # check the input data
    (train, path, settings) = checkAndSetInput!(train, path, settings)
    settings[:detailOfOutput] == "everything" && println("The input has been checked.")

    # prepare the input data
    movingSection = determineCharacteristics(path, train, settings)
    settings[:detailOfOutput] == "everything" && println("The moving section has been prepared.")

    # calculate the train run for oparation mode "minimum running time"
    if settings[:operationModeMinimumRunningTime] || settings[:operationModeMinimumEnergyConsumption]
        (movingSection, drivingCourse) = calculateMinimumRunningTime!(movingSection, settings, train)
        settings[:detailOfOutput] == "everything" && println("The driving course for the shortest running time has been calculated.")

        # accumulate data and create an output dictionary
        output = createOutput(train, settings, path, movingSection, drivingCourse)
        # 30/31 old: output = createOutputDict(train, settings, path, movingSection, drivingCourse)
    else
        output = nothing
        # 30/31 old: output = Dict()
    end #if

    return output
end # function trainRun

# calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime!(movingSection::Dict, settings::Dict, train::Dict)
   CSs::Vector{Dict} = movingSection[:characteristicSections]

   if settings[:massModel] == "homogeneous strip" && settings[:stepVariable] == "v in m/s"
       println("WARNING: ! ! ! TrainRun.jl doesn't work reliably for the mass model homogeneous strip with step size v in m/s. The calculation time can be extremely high when calcutlating paths with steep gradients ! ! !")
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
                         :resistingForceNegative => drivingCourse[end][:F_R] < 0.0,
                         :previousSpeedLimitReached => false, #speedLimitReached, # check already at this position?
                         :speedLimitReached => drivingCourse[end][:v] > CS[:v_limit],
                         :error => false)

    # determine the behavior sections for this characteristic section. It has to be at least one of those BS: "breakFree", "clearing", "accelerating", "cruising", "diminishing", "coasting", "braking" or "standstill")
    while !stateFlags[:endOfCSReached] # s < s_exit
        if !stateFlags[:brakingStartReached] # s+s_braking < s_exit
            if !stateFlags[:tractionDeficit]
                if drivingCourse[end][:F_T] >  drivingCourse[end][:F_R] && drivingCourse[end][:v] == 0.0
                    (CS, drivingCourse, stateFlags) = addBreakFreeSection!(CS, drivingCourse, stateFlags, settings, train, CSs)

                elseif stateFlags[:previousSpeedLimitReached]
                    (CS, drivingCourse, stateFlags) = addClearingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)

                elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R] && !stateFlags[:speedLimitReached]
                    (CS, drivingCourse, stateFlags) = addAcceleratingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)

                elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R] && !stateFlags[:speedLimitReached]
                    # cruise only one step
                    if settings[:stepVariable] =="s in m"
                        s_cruising = settings[:stepSize]
                    elseif settings[:stepVariable] =="t in s"
                        s_cruising = calc_Δs_with_Δt(settings[:stepSize], drivingCourse[end][:a], drivingCourse[end][:v])
                    elseif settings[:stepVariable] =="v in m/s"
                        s_cruising = train[:length]/(10.0) # TODO which step size should be used?
                    end
                    (CS, drivingCourse, stateFlags) = addCruisingSection!(CS, drivingCourse, stateFlags, s_cruising, settings, train, CSs, "cruising")

                elseif  drivingCourse[end][:F_R] < 0 && stateFlags[:speedLimitReached]
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                    s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                    if s_cruising > 0.0
                        (CS, drivingCourse, stateFlags) = addCruisingSection!(CS, drivingCourse, stateFlags, s_cruising, settings, train, CSs, "downhillBraking")
                    else
                        stateFlags[:brakingStartReached] = true
                    end

                elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R] || stateFlags[:speedLimitReached]
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                    s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                    if s_cruising > 0.0  # TODO: define a minimum cruising length?
                        (CS, drivingCourse, stateFlags) = addCruisingSection!(CS, drivingCourse, stateFlags, s_cruising, settings, train, CSs, "cruising")
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
