#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

# Calculate the running time of a train run on a path with special settings with information from the corresponding YAML files with the file paths `trainDirectory`, `pathDirectory`, `settingsDirectory`.

# calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime!(movingSection::Dict, settings::Settings, train::Train)
   CSs::Vector{Dict} = movingSection[:characteristicSections]

   if settings.massModel == :homogeneous_strip && settings.stepVariable == speed
       println("WARNING: ! ! ! TrainRun.jl doesn't work reliably for the mass model homogeneous strip with step size v in m/s. The calculation time can be extremely high when calcutlating paths with steep gradients ! ! !")
   end

   startingPoint=DataPoint()
   startingPoint[:i]=1
   startingPoint[:s]=CSs[1][:s_entry]
   calculateForces!(startingPoint, CSs, 1, "default", train, settings.massModel) # traction effort and resisting forces (in N)
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
       s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking)
       calculateForces!(drivingCourse[end], CSs, CS[:id], "default", train, settings.massModel)     # tractive effort and resisting forces (in N)

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
                    if settings.stepVariable == :distance
                        s_cruising = settings.stepSize
                    elseif settings.stepVariable == time
                        s_cruising = calc_Δs_with_Δt(settings.stepSize, drivingCourse[end][:a], drivingCourse[end][:v])
                    elseif settings.stepVariable == velocity
                        s_cruising = train.length/(10.0) # TODO which step size should be used?
                    end
                    (CS, drivingCourse, stateFlags) = addCruisingSection!(CS, drivingCourse, stateFlags, s_cruising, settings, train, CSs, "cruising")

                elseif  drivingCourse[end][:F_R] < 0 && stateFlags[:speedLimitReached]
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking)
                    s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                    if s_cruising > 0.0
                        (CS, drivingCourse, stateFlags) = addCruisingSection!(CS, drivingCourse, stateFlags, s_cruising, settings, train, CSs, "downhillBraking")
                    else
                        stateFlags[:brakingStartReached] = true
                    end

                elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R] || stateFlags[:speedLimitReached]
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking)
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

"""
    calculateTractiveEffort(v, tractiveEffortVelocityPairs)

Calculate the trains tractive effort with the `tractiveEffortVelocityPairs` dependend on the velocity `v`.

...
# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `tractiveEffortVelocityPairs::Array{}`: the trains pairs for velocity in m/s and tractive effort in N as one array containing an array for each pair.
...

# Examples
```julia-repl
julia> calculateTractiveEffort(20.0, [(0.0, 180000), (20.0, 100000), (40.0, 60000), (60.0, 40000), (80.0, 30000)])
100000

julia> calculateTractiveEffort(30.0, [(0.0, 180000), (20.0, 100000), (40.0, 60000), (60.0, 40000), (80.0, 30000)])
80000
```
"""
function calculateTractiveEffort(v::AbstractFloat, tractiveEffortVelocityPairs::Array{})
    for row in 1:length(tractiveEffortVelocityPairs)
        nextPair = tractiveEffortVelocityPairs[row]
        if  nextPair[1] == v
            return nextPair[2]
        elseif nextPair[1] > v
            # interpolate for a straight line between the two surrounding points with the formula: F=(v-v_(row-1))*(F_row-F_(row-1))/(v_row-v_(row-1))+F_(row-1)
            previousPair = tractiveEffortVelocityPairs[row-1]
            F_T_interpolation = (v-previousPair[1]) * (nextPair[2]-previousPair[2]) / (nextPair[1]-previousPair[1]) + previousPair[2]
            return F_T_interpolation
        end #if
    end #for
    # if v gets higher than the velocities in tractiveEffortVelocityPairs the last tractive effort will be used
        # TODO: also an extrapolation could be used
    return tractiveEffortVelocityPairs[end][2]
end #function calculateTractiveEffort

"""
calculate and return the path resistance dependend on the trains position and mass model
"""
function calculatePathResistance(CSs::Vector{Dict}, csId::Integer, s::Real, massModel, train::Train)

    if massModel == :mass_point
        pathResistance = calcForceFromCoefficient(CSs[csId][:r_path], train.m_train_full)
    elseif massModel == :homogeneous_strip
        pathResistance = 0.0
        s_rear = s - train.length     # position of the rear of the train
        while csId > 0 && s_rear < CSs[csId][:s_exit]
            pathResistance = pathResistance + (min(s, CSs[csId][:s_exit]) - max(s_rear, CSs[csId][:s_entry])) / train.length * calcForceFromCoefficient(CSs[csId][:r_path], train.m_train_full)
            csId = csId-1
            if csId == 0
                # TODO: currently for values  < movingSection[:s_entry] the values of movingSection[:s_entry]  will be used
                return pathResistance + (CSs[1][:s_entry] - s_rear) / train.length * calcForceFromCoefficient(CSs[1][:r_path], train.m_train_full)
            end #if
        end #while
    end #if

    return pathResistance
end #function calculatePathResistance

"""
calculate and return tractive and resisting forces for a data point
"""
function calculateForces!(dataPoint::Dict,  CSs::Vector{Dict}, csId::Integer, bsType::String, train::Train, massModel)
    # calculate resisting forces
    dataPoint[:R_traction] = calcTractionUnitResistance(dataPoint[:v], train)
    dataPoint[:R_wagons] = calcWagonsResistance(dataPoint[:v], train)
    dataPoint[:R_train] = dataPoint[:R_traction] + dataPoint[:R_wagons]
    dataPoint[:R_path] = calculatePathResistance(CSs, csId, dataPoint[:s], massModel, train)
    dataPoint[:F_R] = dataPoint[:R_train] + dataPoint[:R_path]

    # calculate tractive effort
    if bsType == "braking" || bsType == "coasting"
        dataPoint[:F_T] = 0.0
    elseif bsType == "cruising"
        dataPoint[:F_T] = min(max(0.0, dataPoint[:F_R]), calculateTractiveEffort(dataPoint[:v], train.tractiveEffort))
    else # bsType == "accelerating" || bsType == "diminishing" || 'default'
        dataPoint[:F_T] = calculateTractiveEffort(dataPoint[:v], train.tractiveEffort)
    end

    return dataPoint
end #function calculateForces!


"""
TODO
"""
function moveAStep(previousPoint::Dict, stepVariable::Symbol, stepSize::Real, csId::Integer)
    # stepSize is the currentStepSize depending on the accessing function
    # TODO: csId is only for error messages. Should it be removed?
    #= 08/31 TODO: How to check if the train stopps during this step? I should throw an error myself that I catch in higher hierarchies.    =#

    # create the next data point
    newPoint = DataPoint()
    newPoint[:i] = previousPoint[:i]+1         # identifier

    # calculate s, t, v, E
    if stepVariable == :distance                                             # distance step method
        newPoint[:Δs] = stepSize                                                    # step size (in m)
        if previousPoint[:a] == 0.0
            if previousPoint[:v] == 0.0
                error("ERROR: The train tries to cruise at v=0.0 m/s at s=",previousPoint[:s]," in CS",csId,".")
            end
           newPoint[:Δt] = calc_Δt_with_constant_v(newPoint[:Δs], previousPoint[:v])    # step size (in s)
           newPoint[:Δv] = 0.0                                                          # step size (in m/s)
        else
            # check if the parts of the following square roots will be <0.0 in the functions calc_Δt_with_Δs and calc_Δv_with_Δs
            squareRootPartIsNegative = (previousPoint[:v]/previousPoint[:a])^2+2*newPoint[:Δs]/previousPoint[:a] < 0.0 || previousPoint[:v]^2+2*newPoint[:Δs]*previousPoint[:a] < 0.0
            if previousPoint[:a] < 0.0 && squareRootPartIsNegative
                error("ERROR: The train stops during the accelerating section in CS",csId," because the tractive effort is lower than the resistant forces.",
                "       Before the stop the last point has the values s=",previousPoint[:s]," m,  v=",previousPoint[:v]," m/s,  a=",previousPoint[:a]," m/s^2,",
                "       F_T=",previousPoint[:F_T]," N,  R_traction=",previousPoint[:R_traction]," N,  R_wagons=",previousPoint[:R_wagons]," N,  R_path=",previousPoint[:R_path]," N.")
            end
            newPoint[:Δt] = calc_Δt_with_Δs(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in s)
            newPoint[:Δv] = calc_Δv_with_Δs(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in m/s)
        end

    elseif stepVariable == :time                                                              # time step method
        newPoint[:Δt] = stepSize                                                                     # step size (in s)
        newPoint[:Δs] = calc_Δs_with_Δt(newPoint[:Δt], previousPoint[:a], previousPoint[:v])        # step size (in m)
        newPoint[:Δv] = calc_Δv_with_Δt(newPoint[:Δt], previousPoint[:a])                           # step size (in m/s)

    elseif stepVariable  == :velocity                                                            # velocity step method
        if previousPoint[:a] == 0.0
            if previousPoint[:v] == 0.0
                error("ERROR: The train tries to cruise at v=0.0 m/s at s=",previousPoint[:s]," in CS",csId,".")
            end
           newPoint[:Δs] = stepSize                                                     # step size (in m)
            # TODO what is the best default step size for constant v? define Δs or Δt?
           newPoint[:Δt] = calc_Δt_with_constant_v(newPoint[:Δs], previousPoint[:v])    # step size (in s)
           newPoint[:Δv] = 0.0                                                          # step size (in m/s)
        else
            newPoint[:Δv] = stepSize * sign(previousPoint[:a])                                          # step size (in m/s)
            newPoint[:Δs] = calc_Δs_with_Δv(newPoint[:Δv], previousPoint[:a], previousPoint[:v])        # step size (in m)
            newPoint[:Δt] = calc_Δt_with_Δv(newPoint[:Δv], previousPoint[:a])                           # step size (in s)
        end
    end #if

    newPoint[:s] = previousPoint[:s] + newPoint[:Δs]                    # position (in m)
    newPoint[:t] = previousPoint[:t] + newPoint[:Δt]                    # point in time (in s)
    newPoint[:v] = previousPoint[:v] + newPoint[:Δv]                    # velocity (in m/s)
    newPoint[:ΔW] = calc_ΔW(previousPoint[:F_T], newPoint[:Δs])         # mechanical work in this step (in Ws)
    newPoint[:W] = previousPoint[:W] + newPoint[:ΔW]                    # mechanical work (in Ws)
    newPoint[:ΔE] = calc_ΔE(newPoint[:ΔW])                              # energy consumption in this step (in Ws)
    newPoint[:E] = previousPoint[:E] + newPoint[:ΔE]                    # energy consumption (in Ws)


    return newPoint
end #function moveAStep

"""
# if the rear of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
"""
function getCurrentSpeedLimit(CSs::Vector{Dict}, csWithTrainHeadId::Integer, s::Real, trainLength::Real)
    v_limit = CSs[csWithTrainHeadId][:v_limit]
    s_exit = CSs[csWithTrainHeadId][:s_exit]
    if csWithTrainHeadId > 1 && s -trainLength < CSs[csWithTrainHeadId][:s_entry]
        formerCsId = csWithTrainHeadId-1
        while formerCsId > 0 && s -trainLength < CSs[formerCsId][:s_exit]
            if CSs[formerCsId][:v_limit] < v_limit    # TODO: is the position of the train's rear < movingSection[:s_entry], v_limit of the first CS is used
                v_limit = CSs[formerCsId][:v_limit]
                s_exit = CSs[formerCsId][:s_exit]
            end
            formerCsId = formerCsId -1
        end
    end
    currentSpeedLimit = Dict(:v => v_limit, :s_end => s_exit + trainLength)
    return currentSpeedLimit
end #function getCurrentSpeedLimit

"""
?
"""
function getNextPointOfInterest(pointsOfInterest::Vector{Tuple}, s::Real)
    for s_POI in pointsOfInterest
        if s_POI[1] > s
            return s_POI
        end
    end
    error("ERROR in getNextPointOfInterest: There is no POI higher than s=",s," m.")
end #function getNextPointOfInterest


## create a moving section and its containing characteristic sections with secured braking, accelerating and cruising behavior
function determineCharacteristics(path::Path, train::Train, settings::Settings)
    movingSection = MovingSection(path, train.v_limit, train.length)
    movingSection = secureBrakingBehavior!(movingSection, train.a_braking)
    movingSection = secureAcceleratingBehavior!(movingSection, settings, train)
    #movingSection = secureCruisingBehavior!(movingSection, settings, train)

    return movingSection
end #function determineCharacteristics
