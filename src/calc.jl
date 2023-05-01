#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

# Calculate the running time of a train run on a path with special settings with information from the corresponding YAML files with the file paths `trainDirectory`, `pathDirectory`, `settingsDirectory`.

# calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime(CSs::Vector{Dict}, settings::Settings, train::Train)
   startingPoint = SupportPoint()
   startingPoint[:s] = CSs[1][:s_entry]
   if !isempty(CSs[1][:pointsOfInterest]) && CSs[1][:pointsOfInterest][1][:s] == CSs[1][:s_entry]
       startingPoint[:label] =  CSs[1][:pointsOfInterest][1][:label]
   end
   calculateForces!(startingPoint, CSs, 1, "default", train, settings.massModel) # traction effort and resisting forces (in N)
   drivingCourse::Vector{Dict} = [startingPoint]    # List of support points

   for csId in 1:length(CSs)
       CS = CSs[csId]
       # determine the different flags for switching between the states for creating moving phases
       s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
       calculateForces!(drivingCourse[end], CSs, csId, "default", train, settings.massModel)     # tractive effort and resisting forces (in N)

       previousSpeedLimitReached = false
       stateFlags = Dict(:endOfCSReached => drivingCourse[end][:s] > CS[:s_exit],
                         :brakingStartReached => drivingCourse[end][:s] + s_braking == CS[:s_exit],
                         :tractionDeficit => drivingCourse[end][:F_T] < drivingCourse[end][:F_R], # or add another flag for equal forces?
                         :resistingForceNegative => drivingCourse[end][:F_R] < 0.0,
                         :previousSpeedLimitReached => false,
                         :speedLimitReached => drivingCourse[end][:v] > CS[:v_limit],
                         :error => false)

        # determine the behavior sections for this characteristic section. It has to be at least one of those BS: "breakFree", "clearing", "accelerating", "cruising", "diminishing", "coasting", "braking" or "halt")
        while !stateFlags[:endOfCSReached] # s < s_exit
            if stateFlags[:error]
                error("ERROR in calc in CS",csId,":  BS=",drivingCourse[end][:behavior],"  s=",drivingCourse[end][:s],"  s_braking=",s_braking,"  v_limit=",CS[:v_limit],"  v=",drivingCourse[end][:v],"  v_exit=",CS[:v_exit]," with the flags:  endOfCS: ",stateFlags[:endOfCSReached],"    brakingStart: ",stateFlags[:brakingStartReached],"  F_T<F_R: ",stateFlags[:tractionDeficit],"  F_R<0: ",stateFlags[:resistingForceNegative]," v_previousLimit: ",stateFlags[:previousSpeedLimitReached]," v_limit: ",stateFlags[:speedLimitReached]," error: ",stateFlags[:error])
            end

            if !stateFlags[:brakingStartReached] # s+s_braking < s_exit
                if !stateFlags[:tractionDeficit]
                    if drivingCourse[end][:F_T] >  drivingCourse[end][:F_R] && drivingCourse[end][:v] == 0.0
                        (drivingCourse, stateFlags) = addBreakFreeSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

                    elseif stateFlags[:previousSpeedLimitReached]
                        (drivingCourse, stateFlags) = addClearingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

                    elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R] && !stateFlags[:speedLimitReached]
                        (drivingCourse, stateFlags) = addAcceleratingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

                    elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R] && !stateFlags[:speedLimitReached]
                        # cruise only one step
                        if settings.stepVariable == :distance
                            s_cruising = settings.stepSize
                        elseif settings.stepVariable == time
                            s_cruising = Δs_with_Δt(settings.stepSize, drivingCourse[end][:a], drivingCourse[end][:v])
                        elseif settings.stepVariable == velocity
                            s_cruising = train.length/(10.0) # TODO which step size should be used?
                        end
                        (drivingCourse, stateFlags) = addCruisingSection!(drivingCourse, stateFlags, CSs, csId, settings, train, "cruising",  s_cruising)

                    elseif  drivingCourse[end][:F_R] < 0 && stateFlags[:speedLimitReached]
                        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                        s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                        if s_cruising > 0.0
                            (drivingCourse, stateFlags) = addCruisingSection!(drivingCourse, stateFlags, CSs, csId, settings, train, "downhillBraking",  s_cruising)
                        else
                            stateFlags[:brakingStartReached] = true
                        end

                    elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R] || stateFlags[:speedLimitReached]
                        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                        s_cruising = CS[:s_exit] - drivingCourse[end][:s] - s_braking

                        if s_cruising > 1/10^(settings.approxLevel) # TODO: define another minimum cruising length?
                            (drivingCourse, stateFlags) = addCruisingSection!(drivingCourse, stateFlags, CSs, csId, settings, train, "cruising",  s_cruising)
                        else
                            stateFlags[:brakingStartReached] = true
                        end
                    else
                        error()
                    end
                elseif stateFlags[:tractionDeficit]
                    (drivingCourse, stateFlags) = addDiminishingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

                else
                     error()
                end
            else#if !stateFlags[:endOfCSReached] # s < s_exit
                (drivingCourse, stateFlags) = addBrakingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)
            #else
            #   error()
            end

            if CS[:s_exit] - drivingCourse[end][:s] < 1/10^(settings.approxLevel)
                drivingCourse[end][:s] = CS[:s_exit] # round s up to CS[:s_exit]

                # set state flag
                stateFlags[:endOfCSReached] = true
            end
        end
        #if s == s_exit
        #    halt
        #end


           # for testing:   # TODO
           if drivingCourse[end][:s] != CS[:s_exit]
               println("ERROR: In CS", csId," the train run ends at s=",drivingCourse[end][:s]," and not s_exit=",CS[:s_exit])
           end
           if drivingCourse[end][:v] > CS[:v_exit]
               println("ERROR: In CS", csId," the train run ends with v=",drivingCourse[end][:v]," and not with v_exit=",CS[:v_exit])
           end
   end #for

   drivingCourse = addHalt!(drivingCourse, CSs, length(CSs), settings, train)

   return drivingCourse
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
    if v < 0.0
        #println("v=",v)
        return 0.0
    end

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
        pathResistance = forceFromCoefficient(CSs[csId][:r_path], train.m_train_full)
    elseif massModel == :homogeneous_strip
        pathResistance = 0.0
        s_rear = s - train.length     # position of the rear of the train
        while csId > 0 && s_rear < CSs[csId][:s_exit]
            pathResistance = pathResistance + (min(s, CSs[csId][:s_exit]) - max(s_rear, CSs[csId][:s_entry])) / train.length * forceFromCoefficient(CSs[csId][:r_path], train.m_train_full)
            csId = csId-1
            if csId == 0
                # TODO: currently for values  < s_trainrun_start the values of s_trainrun_start  will be used
                return pathResistance + (CSs[1][:s_entry] - s_rear) / train.length * forceFromCoefficient(CSs[1][:r_path], train.m_train_full)
            end #if
        end #while
    end #if

    return pathResistance
end #function calculatePathResistance


"""
calculate and return tractive and resisting forces for a support point
"""
function calculateForces!(supportPoint::Dict,  CSs::Vector{Dict}, csId::Integer, bsType::String, train::Train, massModel)
    # calculate resisting forces
    supportPoint[:R_traction] = tractionUnitResistance(supportPoint[:v], train)
    if train.transportType == :freight
        supportPoint[:R_wagons] = freightWagonsResistance(supportPoint[:v], train)
    elseif train.transportType == :passenger
        supportPoint[:R_wagons] = passengerWagonsResistance(supportPoint[:v], train)
    end
    supportPoint[:R_train] = supportPoint[:R_traction] + supportPoint[:R_wagons]
    supportPoint[:R_path] = calculatePathResistance(CSs, csId, supportPoint[:s], massModel, train)
    supportPoint[:F_R] = supportPoint[:R_train] + supportPoint[:R_path]

    # calculate tractive effort
    if bsType == "braking" || bsType == "coasting" || bsType == "halt"
        supportPoint[:F_T] = 0.0
    elseif bsType == "cruising"
        supportPoint[:F_T] = min(max(0.0, supportPoint[:F_R]), calculateTractiveEffort(supportPoint[:v], train.tractiveEffort))
    else # bsType == "accelerating" || bsType == "diminishing" || 'default'
        supportPoint[:F_T] = calculateTractiveEffort(supportPoint[:v], train.tractiveEffort)
    end

    return supportPoint
end #function calculateForces!


"""
TODO
"""
function moveAStep(previousPoint::Dict, stepVariable::Symbol, stepSize::Real, csId::Integer)
    # stepSize is the currentStepSize depending on the accessing function
    # TODO: csId is only for error messages. Should it be removed?
    #= 08/31 TODO: How to check if the train stopps during this step? I should throw an error myself that I catch in higher hierarchies.    =#

    # create the next support point
    newPoint = SupportPoint()

    # calculate s, t, v, E
    if stepVariable == :distance                                             # distance step method
        Δs = stepSize                                                    # step size (in m)
        if previousPoint[:a] == 0.0
            if previousPoint[:v] == 0.0
                error("ERROR: The train tries to cruise at v=0.0 m/s at s=",previousPoint[:s]," in CS",csId,".")
            end
           Δt = Δt_with_constant_v(Δs, previousPoint[:v])    # step size (in s)
           Δv = 0.0                                                          # step size (in m/s)
        else
            # check if the parts of the following square roots will be <0.0 in the functions Δt_with_Δs and Δv_with_Δs
            squareRootPartIsNegative = (previousPoint[:v]/previousPoint[:a])^2+2*Δs/previousPoint[:a] < 0.0 || previousPoint[:v]^2+2*Δs*previousPoint[:a] < 0.0
            if previousPoint[:a] < 0.0 && squareRootPartIsNegative
                error("ERROR: The train stops during the accelerating section in CS",csId," because the tractive effort is lower than the resistant forces.",
                "       Before the stop the last point has the values s=",previousPoint[:s]," m,  v=",previousPoint[:v]," m/s,  a=",previousPoint[:a]," m/s^2,",
                "       F_T=",previousPoint[:F_T]," N,  R_traction=",previousPoint[:R_traction]," N,  R_wagons=",previousPoint[:R_wagons]," N,  R_path=",previousPoint[:R_path]," N.")
            end
            Δt = Δt_with_Δs(Δs, previousPoint[:a], previousPoint[:v])        # step size (in s)
            Δv = Δv_with_Δs(Δs, previousPoint[:a], previousPoint[:v])        # step size (in m/s)
        end

    elseif stepVariable == :time                                                              # time step method
        Δt = stepSize                                                                     # step size (in s)
        Δs = Δs_with_Δt(Δt, previousPoint[:a], previousPoint[:v])        # step size (in m)
        Δv = Δv_with_Δt(Δt, previousPoint[:a])                           # step size (in m/s)

    elseif stepVariable  == :velocity                                                            # velocity step method
        if previousPoint[:a] == 0.0
            if previousPoint[:v] == 0.0
                error("ERROR: The train tries to cruise at v=0.0 m/s at s=",previousPoint[:s]," in CS",csId,".")
            end
           Δs = stepSize                                                     # step size (in m)
            # TODO what is the best default step size for constant v? define Δs or Δt?
           Δt = Δt_with_constant_v(Δs, previousPoint[:v])    # step size (in s)
           Δv = 0.0                                                          # step size (in m/s)
        else
            Δv = stepSize * sign(previousPoint[:a])                                          # step size (in m/s)
            Δs = Δs_with_Δv(Δv, previousPoint[:a], previousPoint[:v])        # step size (in m)
            Δt = Δt_with_Δv(Δv, previousPoint[:a])                           # step size (in s)
        end
    end #if

    newPoint[:s] = previousPoint[:s] + Δs                    # position (in m)
    newPoint[:t] = previousPoint[:t] + Δt                    # point in time (in s)
    newPoint[:v] = previousPoint[:v] + Δv                    # velocity (in m/s)

    return newPoint
end #function moveAStep


"""
# if the rear of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
"""
function getLowestSpeedLimit(CSs::Vector{Dict}, csWithTrainHeadId::Integer, s::Real, trainLength::Real)
    v_limit = CSs[csWithTrainHeadId][:v_limit]
    s_exit = CSs[csWithTrainHeadId][:s_exit]
    if csWithTrainHeadId > 1 && s -trainLength < CSs[csWithTrainHeadId][:s_entry]
        formerCsId = csWithTrainHeadId-1
        while formerCsId > 0 && s -trainLength < CSs[formerCsId][:s_exit]
            if CSs[formerCsId][:v_limit] < v_limit    # TODO: is the position of the train's rear < s_trainrun_start, v_limit of the first CS is used
                v_limit = CSs[formerCsId][:v_limit]
                s_exit = CSs[formerCsId][:s_exit]
            end
            formerCsId = formerCsId -1
        end
    end
    lowestSpeedLimit = Dict(:v => v_limit, :s_end => s_exit + trainLength)
    return lowestSpeedLimit
end #function getLowestSpeedLimit


"""
TODO
"""
function getNextPointOfInterest(pointsOfInterest::Vector{NamedTuple}, s::Real)
    for POI in pointsOfInterest
        if POI[:s] > s
            return POI
        end
    end
    error("ERROR in getNextPointOfInterest: There is no POI higher than s=",s," m.")
end #function getNextPointOfInterest


## create vectors with the moving section's points of interest and with the characteristic sections with secured braking and accelerating behavior
function determineCharacteristics(path::Path, train::Train, settings::Settings)
    # determine the positions of the points of interest depending on the interesting part of the train (front/rear) and the train's length
    poi_positions = []
    pointsOfInterest = NamedTuple[]
    if !isempty(path.poi)
        for POI in path.poi
            s_poi = POI[:station]
            if POI[:measure] == "rear"
                s_poi += train.length
            end
            push!(pointsOfInterest, (s = s_poi, label = POI[:label]) )
            push!(poi_positions,s_poi)
        end
        sort!(pointsOfInterest, by = x -> x[:s])
    end

    # create the characteristic sections of a moving section 'CSs' dependent on the paths attributes
    CSs = Vector{Dict}()
    s_csStart = path.sections[1][:s_start]          # first position (in m)

    for row in 2:length(path.sections)
        previousSection = path.sections[row-1]
        currentSection = path.sections[row]

        speedLimitIsDifferent = min(previousSection[:v_limit], train.v_limit) != min(currentSection[:v_limit], train.v_limit)
        pathResistanceIsDifferent = previousSection[:f_Rp] != currentSection[:f_Rp]
        if speedLimitIsDifferent || pathResistanceIsDifferent
            push!(CSs, CharacteristicSection(s_csStart, previousSection, min(previousSection[:v_limit], train.v_limit), train.length, pointsOfInterest))
            s_csStart = currentSection[:s_start]
        end #if
    end #for
    push!(CSs, CharacteristicSection(s_csStart, path.sections[end], min(path.sections[end][:v_limit], train.v_limit), train.length, pointsOfInterest))

    # secure that the train is able to brake sufficiently and keeps speed limits
    CSs = secureBrakingBehavior!(CSs, train.a_braking, settings.approxLevel)

    return (CSs, poi_positions)
end #function determineCharacteristics
