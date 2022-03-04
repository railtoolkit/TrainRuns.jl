#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

module Behavior

include("./DrivingDynamics.jl")
using .DrivingDynamics

export addBreakFreeSection!, addAcceleratingSection!, addCruisingSection!, addDiminishingSection!, addCoastingSection!, addBrakingSection!, addStandstill!,
# addBrakingSectionInOneStep! is not used in the current version of the tool
calculateForces!, createDataPoint,

# export functions from DrivingDynamics
calcBrakingDistance, calcBrakingStartVelocity


approximationLevel = 6  # value for approximation to intersections TODO further explanation (e.g. approximationLevel = 3 -> with stepSize 10 m the approximation will be calculated accurate on 10 mm ; 1s -> 1 ms; 1 km/h -> 3.6 mm/s)
    # TODO: define it in TrainRun and give it to each function?

## functions for calculating tractive effort and resisting forces
"""
    calculateTractiveEffort(v, tractiveEffortVelocityPairs)

Calculate the trains tractive effort with the `tractiveEffortVelocityPairs` dependend on the velocity `v`.

...
# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `tractiveEffortVelocityPairs::Array{Array{AbstractFloat,1},1}`: the trains pairs for velocity in m/s and tractive effort in N as one array containing an array for each pair.
...

# Examples
```julia-repl
julia> calculateTractiveEffort(20.0, [[0.0, 180000], [20.0, 100000], [40.0, 60000], [60.0, 40000], [80.0, 30000]])
100000

julia> calculateTractiveEffort(30.0, [[0.0, 180000], [20.0, 100000], [40.0, 60000], [60.0, 40000], [80.0, 30000]])
80000
```
"""
function calculateTractiveEffort(v::AbstractFloat, tractiveEffortVelocityPairs)
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
function calculatePathResistance(CSs::Vector{Dict}, csId::Integer, s::Real, massModel::String, train::Dict)
    if massModel == "mass point"
        pathResistance = calcForceFromCoefficient(CSs[csId][:r_path], train[:m_train])
    elseif massModel == "homogeneous strip"
        pathResistance = 0.0
        while csId > 0 && s-train[:length] < CSs[csId][:s_exit]
            pathResistance = pathResistance + (min(s, CSs[csId][:s_exit]) - max(s-train[:length], CSs[csId][:s_entry])) / train[:length] * calcForceFromCoefficient(CSs[csId][:r_path], train[:m_train])
            csId = csId-1
            if csId == 0
                # TODO: currently for values  < movingSection[:s_entry] the values of movingSection[:s_entry]  will be used
                return pathResistance + (CSs[1][:s_entry] - (s-train[:length])) / train[:length] * calcForceFromCoefficient(CSs[1][:r_path], train[:m_train])
            end #if
        end #while
    end #if

    return pathResistance
end #function calculatePathResistance

"""
calculate and return tractive and resisting forces for a data point
"""
function calculateForces!(dataPoint::Dict,  CSs::Vector{Dict}, csId::Integer, bsType::String, train::Dict, massModel::String)
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
        dataPoint[:F_T] = min(max(0.0, dataPoint[:F_R]), calculateTractiveEffort(dataPoint[:v], train[:tractiveEffortVelocityPairs]))
    else # bsType == "accelerating" || bsType == "diminishing" || 'default'
        dataPoint[:F_T] = calculateTractiveEffort(dataPoint[:v], train[:tractiveEffortVelocityPairs])
    end

    return dataPoint
end #function calculateForces!


"""
TODO
"""
function moveAStep(previousPoint::Dict, stepVariable::String, stepSize::Real, csId::Integer)
    # stepSize is the currentStepSize depending on the accessing function
    # TODO: csId is only for error messages. Should it be removed?
    #= 08/31 TODO: How to check if the train stopps during this step? I should throw an error myself that I catch in higher hierarchies.    =#

    # create the next data point
    newPoint = createDataPoint()
    newPoint[:i] = previousPoint[:i]+1         # identifier

    # calculate s, t, v, E
    if stepVariable == "s in m"                                             # distance step method
        newPoint[:Δs] = stepSize                                                    # step size (in m)
        if previousPoint[:a] == 0.0
            if previousPoint[:v] == 0.0
                error("ERROR: The train tries to cruise at v=0.0 m/s at s=",previousPoint[:s]," in CS",csId,".")
            end
           newPoint[:Δt] = calc_Δt_with_constant_v(newPoint[:Δs], previousPoint[:v])    # step size (in s)
           newPoint[:Δv] = 0.0                                                          # step size (in m/s)
        else
            if previousPoint[:a] < 0.0 && ((previousPoint[:v]/previousPoint[:a])^2+2*newPoint[:Δs]/previousPoint[:a])<0.0 || (previousPoint[:v]^2+2*newPoint[:Δs]*previousPoint[:a])<0.0  # checking if the parts of the following square roots will be <0.0
                error("ERROR: The train stops during the accelerating section in CS",csId," because the tractive effort is lower than the resistant forces.",
                "       Before the stop the last point has the values s=",previousPoint[:s]," m,  v=",previousPoint[:v]," m/s,  a=",previousPoint[:a]," m/s^2,",
                "       F_T=",previousPoint[:F_T]," N,  R_traction=",previousPoint[:R_traction]," N,  R_wagons=",previousPoint[:R_wagons]," N,  R_path=",previousPoint[:R_path]," N.")
            end
            newPoint[:Δt] = calc_Δt_with_Δs(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in s)
            newPoint[:Δv] = calc_Δv_with_Δs(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in m/s)
        end

    elseif stepVariable == "t in s"                                                              # time step method
        newPoint[:Δt] = stepSize                                                                     # step size (in s)
        newPoint[:Δs] = calc_Δs_with_Δt(newPoint[:Δt], previousPoint[:a], previousPoint[:v])        # step size (in m)
        newPoint[:Δv] = calc_Δv_with_Δt(newPoint[:Δt], previousPoint[:a])                           # step size (in m/s)

    elseif stepVariable  == "v in m/s"                                                            # velocity step method
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


function considerFormerSpeedLimit!(CS::Dict, drivingCourse::Vector{Dict}, acceleratingSection::Dict, settings::Dict, train::Dict, CSs::Vector{Dict}, currentSpeedLimit::Dict)
    # if a former speed limit has been exceeded the accelerating steps of this CS will be removed and a clearing section will be inserted before accelerating
    if drivingCourse[end][:v] > currentSpeedLimit[:v]
        # delete existing clearing section that is not long enough
        while drivingCourse[end][:s] > get(CS[:behaviorSections], :clearing, acceleratingSection)[:s_entry]
            pop!(drivingCourse)
        end

        if haskey(CS[:behaviorSections], :clearing)
            CS[:t] = CS[:t]-CS[:behaviorSections][:clearing][:t]              # reducing the total running time (in s)
            CS[:E] = CS[:E]-CS[:behaviorSections][:clearing][:E]              # reducing the total energy consumption (in Ws)
            delete!(CS[:behaviorSections], :clearing)
        end

        # create a (new and longer) clearing section
        s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
        s_clearing = min(CS[:s_exit]-drivingCourse[end][:s]-s_braking, currentSpeedLimit[:s_end] - drivingCourse[end][:s])
        if s_clearing > 0.0
            (CS, drivingCourse, brakingStartReached) = addCruisingSection!(CS, drivingCourse, s_clearing, settings, train, CSs, "clearing")
        else
            error("ERROR: clearing <=0.0 although it has to be >0.0 in CS ",CS[:id])
        end

        calculateForces!(drivingCourse[end], CSs, CS[:id], acceleratingSection[:type], train, settings[:massModel])

        # reset the acceleratingSection
        acceleratingSection = createBehaviorSection("accelerating", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])

        if  drivingCourse[end][:s] + s_braking >= CS[:s_exit]
            CS[:v_peak] = drivingCourse[end][:v]
        end
    end

    return (CS, drivingCourse, acceleratingSection)
end # function considerFormerSpeedLimit!

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

function getNextPointOfInterest(pointsOfInterest::Vector{Real}, s::Real)
    for POI in pointsOfInterest
        if POI > s
            return POI
        end
    end
    error("ERROR in getNextPointOfInterest: There is no POI higher than s=",s," m.")
end #function getNextPointOfInterest

## This function calculates the data points of the breakFree section.
# Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for breakFree if needed.
# Info: currently the values of the breakFree section will be calculated like in the accelerating section
function addBreakFreeSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}, ignoreBraking::Bool)
    if drivingCourse[end][:v]==0.0 && drivingCourse[end][:s]<CS[:s_exit]
        BS = createBehaviorSection("breakFree", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, CS[:id], "accelerating", train, settings[:massModel])    # currently the tractive effort is calculated like in the accelerating section

        # calculate the breakFree section with calculating the accelerating section and just using the first step and removing the rest
        try (CS, drivingCourse, brakingStartReached) = addAcceleratingSection!(CS, drivingCourse, settings, train, CSs, false)
        catch(acceleratingError)
            println("This error happened during the break free phase that is using the accelerating function:")
            rethrow(acceleratingError)
        end

        # delete every dataPoint except the first two
        while drivingCourse[end][:i] > drivingCourse[BS[:dataPoints][1]][:i] +1
            pop!(drivingCourse)
        end

        # change the accelerating data to break free
        drivingCourse[end-1][:behavior] = BS[:type]
        drivingCourse[end][:behavior] = BS[:type]
        push!(BS[:dataPoints], drivingCourse[end][:i])

        # remove the accelerating section from the CS
        CS[:t] = CS[:t] - get(CS[:behaviorSections], :accelerating, Dict(:t=>0.0))[:t]         # total running time (in s)
        CS[:E] = CS[:E] - get(CS[:behaviorSections], :accelerating, Dict(:E=>0.0))[:E]         # total energy consumption (in Ws)
        delete!(CS[:behaviorSections], :accelerating)

        # calculate the accumulated breakFree section information
        merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                        :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                        :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                        :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                        :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

        CS[:t] = CS[:t] + BS[:t]         # total running time (in s)
        CS[:E] = CS[:E] + BS[:E]         # total energy consumption (in Ws)

        merge!(CS[:behaviorSections], Dict(:breakFree => BS))
    end # else: return the characteristic section without a breakFree section
    return (CS, drivingCourse)
end #function addBreakFreeSection!


## This function calculates the data points of the accelerating section.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the accelerating section
function addAcceleratingSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}, ignoreBraking::Bool)
    #=if drivingCourse would also be part of movingSectiong: function addAcceleratingSection!(movingSection::Dict, csId::Integer, settings::Dict, train::Dict, ignoreBraking::Bool)
      CSs = movingSection[:characteristicSections]
      CS = CSs[csId]
      drivingCourse = movingSection[:drivingCourse]=#

    calculateForces!(drivingCourse[end], CSs, CS[:id], "accelerating", train, settings[:massModel])

    if ignoreBraking
        s_braking = 0.0
    else
        s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
    end

    # conditions for the accelerating section
    targetSpeedReached = drivingCourse[end][:v] >= CS[:v_peak]
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]

    if ignoreBraking
        s_braking = 0.0
    else
        s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
    end
    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]

    # use the conditions for the accelerating section
    if !targetSpeedReached && !trainAtEnd && tractionSurplus
        BS = createBehaviorSection("accelerating", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]
        while !targetSpeedReached && !trainAtEnd && tractionSurplus && !brakingStartReached
            currentStepSize = settings[:stepSize]   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train[:length])

            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                if !ignoreBraking
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                end

                while drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:v] <= currentSpeedLimit[:v] && !brakingStartReached && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]      # as long as s_i + s_braking < s_CSexit
                # 03/02 old: while drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:v] <= currentSpeedLimit[:v] && drivingCourse[end][:s] +s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]      # as long as s_i + s_braking < s_CSexit
                # 02/23 old: while drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:s] +s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]      # as long as s_i + s_braking < s_CSexit
                if drivingCourse[end][:s] >= currentSpeedLimit[:s_end]
                    # could be asked after creating an data point. This way here prevents even a minimal exceedance of speed limit will be noticed. On the other hand the train cruises possibly a little to long
                    currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train[:length])
                end


                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                    # create the next data point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = BS[:type]
                    push!(BS[:dataPoints], drivingCourse[end][:i])

                    calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])
                    if !ignoreBraking
                        s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                        brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
                    end
                end #while

                if CS[:id]==0
                    testFlag = true
                else
                    testFlag = false     # for testing
                end

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                    if drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," <= F_R=",drivingCourse[end][:F_R])    # for testing
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPOI=",nextPointOfInterest)    # for testing
                        if settings[:stepVariable] == "s in m"
                            currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > CS[:v_peak]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_peak=",CS[:v_peak])    # for testing
                        if settings[:stepVariable] == "v in m/s"
                            currentStepSize = CS[:v_peak]-drivingCourse[end-1][:v]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > currentSpeedLimit[:v]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_limitCurrent=",currentSpeedLimit[:v])    # for testing
                        (CS, drivingCourse, BS) = considerFormerSpeedLimit!(CS, drivingCourse, BS, settings, train, CSs, currentSpeedLimit)
                        break
                        # TODO: think about alternative: handle the the position where the rear of the train leaves a CS similar to a POI. Then it will be calculated exactly and even a minimal exceedance of speed limit will be noticed

                    elseif drivingCourse[end][:s] +s_braking == CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                        brakingStartReached = true
                        if s_braking == 0.0
                            trainAtEnd = true
                        end
                        break

                    elseif drivingCourse[end][:v] == CS[:v_peak]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," == v_peak=",CS[:v_peak])    # for testing
                        targetSpeedReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," == nextPOI=",nextPointOfInterest)    # for testing
                        break

                    else
                        println("v=",drivingCourse[end][:v],"   v_peak= ", CS[:v_peak] , "  v_cLimit=", currentSpeedLimit[:v])
                        println("s=" ,drivingCourse[end][:s],"   s_exit=", CS[:s_exit], "   s+s_braking=", drivingCourse[end][:s] +s_braking,"   nextPOI=",nextPointOfInterest)
                        println("F_T=",drivingCourse[end][:F_T] ,"   F_R=", drivingCourse[end][:F_R])

                        error("ERROR at accelerating section: With the step variable ",settings[:stepVariable]," the while loop will be left although v<v_peak and s<s_exit in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])
                    brakingStartReached = false

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] > CS[:v_peak]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_peak=",CS[:v_peak])    # for testing
                        targetSpeedReached = true
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])
                        brakingStartReached = false

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        brakingStartReached = true
                        if s_braking > 0.0
                            pop!(drivingCourse)
                            pop!(BS[:dataPoints])
                        else
                            drivingCourse[end][:s] = CS[:s_exit] # round s down to CS[:s_exit]
                            drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]
                        end

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:s]," > nextPointOfInterest",nextPointOfInterest)    # for testing
                        drivingCourse[end][:s] = nextPointOfInterest # round s down to nextPointOfInterest
                        drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," <= F_R=",drivingCourse[end][:F_R])    # for testing
                        tractionSurplus = false

                    elseif drivingCourse[end][:v] > currentSpeedLimit[:v]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_limitCurrent=",currentSpeedLimit[:v])    # for testing
                        (CS, drivingCourse, BS) = considerFormerSpeedLimit!(CS, drivingCourse, BS, settings, train, CSs, currentSpeedLimit)
                        # TODO: think about alternative: handle the the position where the rear of the train leaves a CS similar to a POI. Then it will be calculated exactly and the train start accelerating immediately and not only at the next data point.

                    else
                        if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                            testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," else case and there: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                            brakingStartReached = true
                        end
                    end

                    # TODO is it possible to put this into to the if-fork?
                    if drivingCourse[end][:s] == CS[:s_exit]
                        trainAtEnd = true
                    end
                end
            end #for

            if drivingCourse[end][:s] == CS[:s_exit]
                trainAtEnd = true
            end
        end #while

        if length(BS[:dataPoints]) > 1
            # calculate the accumulated accelerating section information
            merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                            :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                            :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                            :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                            :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

            CS[:v_peak] = max(drivingCourse[end][:v], CS[:v_entry])      # setting v_peak to the last data points velocity which is the highest reachable value in this characteristic section or to v_entry in case it is higher when running on a path with high resistances
            CS[:t] = CS[:t] + BS[:t]       # total running time (in s)
            CS[:E] = CS[:E] + BS[:E]       # total energy consumption (in Ws)

            mergeBehaviorSection!(CS[:behaviorSections], BS)
        end
    end

    return (CS, drivingCourse, brakingStartReached)
end #function addAcceleratingSection!


## This function calculates the data points of the cruising section.
#   Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for cruising if needed.
function addCruisingSection!(CS::Dict, drivingCourse::Vector{Dict}, s_cruising::Real, settings::Dict, train::Dict, CSs::Vector{Dict}, cruisingType::String)
    # traction effort and resisting forces (in N)
    calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])

    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]

    if drivingCourse[end][:v]>0.0 && drivingCourse[end][:v]<=CS[:v_peak] && !brakingStartReached && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
    # old: if drivingCourse[end][:v]>0.0 && drivingCourse[end][:v]<=CS[:v_peak] && drivingCourse[end][:s]<CS[:s_exit] && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
        BS = createBehaviorSection(cruisingType, drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        # TODO: necessary?
        s_cruising = min(s_cruising, CS[:s_exit]-BS[:s_entry])

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel]) # TODO: or give BS[:type] instead of "cruising"?

        if settings[:massModel]=="homogeneous strip" && CS[:id] > 1
            # conditions for cruising section
            trainInPreviousCS = drivingCourse[end][:s] < CS[:s_entry] + train[:length]
            trainAtEnd = drivingCourse[end][:s] >= BS[:s_entry] +s_cruising
            tractionSurplus = drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]

            # use the conditions for the cruising section
            while trainInPreviousCS && !trainAtEnd && tractionSurplus
                currentStepSize = settings[:stepSize]
                nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

                for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                    while drivingCourse[end][:s] < CS[:s_entry] + train[:length] && drivingCourse[end][:s] < BS[:s_entry] +s_cruising && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T]>=drivingCourse[end][:F_R]
                     # the tractive effort is lower than the resisiting forces and the train has use the highest possible effort to try to stay at v_peak OR the mass model homogeneous strip is used and parts of the train are still in former CS
                      #TODO: maybe just consider former CS with different path resistance?

                        # acceleration (in m/s^2):
                        drivingCourse[end][:a] = 0.0

                        # create the next data point
                        if settings[:stepVariable] =="s in m" || settings[:stepVariable] =="t in s"
                            push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                        else
                            push!(drivingCourse, moveAStep(drivingCourse[end], "s in m", train[:length]/(10.0^cycle), CS[:id])) # TODO which step size should be used?
                        end
                        drivingCourse[end][:behavior] = BS[:type]
                        push!(BS[:dataPoints], drivingCourse[end][:i])

                        # traction effort and resisting forces (in N)
                        calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])
                    end #while

                    # check which limit was reached and adjust the currentStepSize for the next cycle
                    if cycle < approximationLevel+1
                        if drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                            currentStepSize = settings[:stepSize] / 10.0^cycle

                        elseif drivingCourse[end][:s] > nextPointOfInterest
                            if settings[:stepVariable] == "s in m"
                                currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                            else
                                currentStepSize = settings[:stepSize] / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] > BS[:s_entry] + s_cruising # TODO also the following? drivingCourse[end][:s] > CSs[CS[:id]][:s_entry] + train[:length]))
                            if settings[:stepVariable] == "s in m"
                                currentStepSize=BS[:s_entry] + s_cruising-drivingCourse[end-1][:s]
                            else
                                currentStepSize = settings[:stepSize] / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] == BS[:s_entry] + s_cruising # || drivingCourse[end][:s]==CS[:s_exit]
                            trainAtEnd = true
                            break

                        elseif drivingCourse[end][:s] >= CS[:s_entry] + train[:length]
                            trainInPreviousCS = false
                            break

                        elseif drivingCourse[end][:s] == nextPointOfInterest
                            break

                        else # TODO copied from addAcceleratingSection -> probably not needed here !?
                            error("ERROR at cruising section: With the step variable ",settings[:stepVariable]," the while loop will be left although the if cases don't apply in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                        end

                        # delete last data point for recalculating the last step with reduced step size
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    else # if the level of approximation is reached
                        if drivingCourse[end][:s] > nextPointOfInterest
                            drivingCourse[end][:s] = nextPointOfInterest # round s down to nextPointOfInterest
                            drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]
                        elseif drivingCourse[end][:s] > BS[:s_entry]+s_cruising
                            trainAtEnd = true
                            if BS[:type] != "clearing"
                                pop!(drivingCourse)
                                pop!(BS[:dataPoints])
                            end
                        elseif drivingCourse[end][:s] == BS[:s_entry]+s_cruising
                            trainAtEnd = true
                            break
                        elseif drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                            tractionSurplus = false

                        else

                        end
                    end
                end #for
            end #while
        end #if

        # TODO oder soll das lieber nach oben in den else des letzten Durchlaufs. Noch mal genauer ansehen, ob hier was doppelt gemoppelt ist
        #if drivingCourse[end][:s] < BS[:s_entry]+s_cruising && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
        while drivingCourse[end][:s] < BS[:s_entry]+s_cruising && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
            nextPointOfInterest = min(BS[:s_entry]+s_cruising, getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s]))
            drivingCourse[end][:a] = 0.0    # acceleration (in m/s^2)

            # calculate the remaining cruising way
            #s_cruisingRemaining=BS[:s_entry] + s_cruising-drivingCourse[end][:s]
            s_cruisingRemaining = min(nextPointOfInterest -drivingCourse[end][:s], BS[:s_entry] +s_cruising -drivingCourse[end][:s])

            # create the next data point
            push!(drivingCourse, moveAStep(drivingCourse[end], "s in m", s_cruisingRemaining, CS[:id]))
            drivingCourse[end][:behavior] = BS[:type]
            push!(BS[:dataPoints], drivingCourse[end][:i])

            calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])
        end #while

        # TODO: realize this better inside the upper loops?
        s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
        brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]

        # calculate the accumulated cruising section information
        merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                        :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                        :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                        :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                        :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

        CS[:t] = CS[:t] + BS[:t]       # total running time (in s)
        CS[:E] = CS[:E] + BS[:E]       # total energy consumption (in Ws)

        mergeBehaviorSection!(CS[:behaviorSections], BS)
    end # else: return the characteristic section without a cruising section

    return (CS, drivingCourse, brakingStartReached)
end #function addCruisingSection!


## This function calculates the data points for diminishing run when using maximum tractive effort and still getting slower
function addDiminishingSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    calculateForces!(drivingCourse[end], CSs, CS[:id], "diminishing", train, settings[:massModel])
    # conditions for diminishing section
    targetSpeedReached = drivingCourse[end][:v] <= 0.0
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]

    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]

    # use the conditions for the diminishing section
    if !tractionSurplus && !targetSpeedReached && !trainAtEnd
        BS = createBehaviorSection("diminishing", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        while !tractionSurplus && !targetSpeedReached && !trainAtEnd && !brakingStartReached
            currentStepSize=settings[:stepSize]   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                #while drivingCourse[end][:F_T] <= drivingCourse[end][:F_R] && drivingCourse[end][:s] + s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:v]>0.0       # as long as s_i + s_braking < s_end
                #while drivingCourse[end][:F_T] < drivingCourse[end][:F_R] && drivingCourse[end][:s] + s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:v]>0.0       # as long as s_i + s_braking < s_end
                while drivingCourse[end][:F_T] < drivingCourse[end][:F_R] && !brakingStartReached && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:v]>0.0       # as long as s_i + s_braking < s_end
                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                    # create the next data point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = BS[:type]
                    push!(BS[:dataPoints], drivingCourse[end][:i])

                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
                    calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])
                end #while

                if CS[:id]==0
                    testFlag = true
                else
                    testFlag = false     # for testing
                end

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                    if drivingCourse[end][:v] < 0.0
                        if settings[:stepVariable] == "v in m/s"
                            currentStepSize = drivingCourse[end-1][:v]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end
                    elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," > F_R=",drivingCourse[end][:F_R])    # for testing
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPOI=",nextPointOfInterest)    # for testing
                        if settings[:stepVariable] == "s in m"
                            currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                        brakingStartReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," == nextPOI=",nextPointOfInterest)    # for testing
                        break

                    elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," == F_R=",drivingCourse[end][:F_R])    # for testing
                        tractionSurplus = true
                        break

                    elseif drivingCourse[end][:v] == 0.0
                        targetSpeedReached = true
                        error("ERROR: The train stops during diminishing run in CS",CS[:id]," at position s=",drivingCourse[end][:s]," m because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    else
                        error("ERROR during diminishing run: With the step variable ",settings[:stepVariable]," the while loop will be left although s+s_braking<s_exit && v>0.0  in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])
                    brakingStartReached = false

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: v=", drivingCourse[end][:v]," <= 0.0")    # for testing
                        targetSpeedReached = true
                        # push!(BS[:dataPoints], drivingCourse[end][:i])
                        error("ERROR: The train stops during diminishing run in CS",CS[:id]," because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        brakingStartReached = true
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest=",nextPointOfInterest)    # for testing
                        drivingCourse[end][:s] = nextPointOfInterest # round s down to nextPointOfInterest
                        drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]

                    elseif drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," >= F_R=", drivingCourse[end][:F_R])    # for testing
                        tractionSurplus = true
                        break

                    else
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: else with v=", drivingCourse[end][:v]," > 0.0   and F_T=", drivingCourse[end][:F_T]," <= F_R=", drivingCourse[end][:F_R])    # for testing
                        #println("     and s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," <= s_exit=",CS[:s_exit])    # for testing
                        #println("     and s=", drivingCourse[end][:s]," <= nextPointOfInterest=",nextPointOfInterest)    # for testing

                        if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                            brakingStartReached = true
                        end
                    end #if

                    # TODO is it possible to put this into to the if-fork?
                    if drivingCourse[end][:s] == CS[:s_exit]
                        trainAtEnd = true
                    end
                end #if
            end #for
        end #while

        if length(BS[:dataPoints]) > 1 # TODO: necessary? May it be possible that there is no diminishing because braking has to start?
            # calculate the accumulated diminishing section information
            merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                            :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                            :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                            :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                            :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

            CS[:t] = CS[:t] + BS[:t]             # total running time (in s)
            CS[:E] = CS[:E] + BS[:E]             # total energy consumption (in Ws)

            mergeBehaviorSection!(CS[:behaviorSections], BS)
        end
    end

    return (CS, drivingCourse, brakingStartReached)
end #function addDiminishingSection!


## This function calculates the data points of the coasting section.
# Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the coasting section
function addCoastingSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    # TODO: if the rear of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
            # with getCurrentSpeedLimit

    # conditions for coasting section
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]

    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]

    # use the conditions for the coasting section
    if !targetSpeedReached && !trainAtEnd
       BS = createBehaviorSection("coasting", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
       drivingCourse[end][:behavior] = BS[:type]

       while !targetSpeedReached && !trainAtEnd && !brakingStartReached
           currentStepSize=settings[:stepSize]  # initialize the step size that can be reduced near intersections
           nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

           for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                while drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:v] <= CS[:v_peak] && !brakingStartReached && drivingCourse[end][:s] < nextPointOfInterest # as long as s_i + s_braking < s_end
                   # traction effort and resisting forces (in N):
                   calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

                   # acceleration (in m/s^2):
                   drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                   # create the next data point
                   push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                   drivingCourse[end][:behavior] = BS[:type]
                   push!(BS[:dataPoints], drivingCourse[end][:i])

                   s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                   brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
                end # while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                   if drivingCourse[end][:s] + s_braking > CS[:s_exit]
                       currentStepSize = settings[:stepSize] / 10.0^cycle

                   elseif drivingCourse[end][:s] > nextPointOfInterest
                       if settings[:stepVariable] == "s in m"
                           currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                       else
                           currentStepSize = settings[:stepSize] / 10.0^cycle
                       end

                   elseif drivingCourse[end][:v] < CS[:v_exit]  # TODO: if accelereation and coasting functions will be combined this case is only for coasting
                        if settings[:stepVariable] == "v in m/s"
                            currentStepSize = drivingCourse[end-1][:v] - CS[:v_exit]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end
                   elseif drivingCourse[end][:v] > CS[:v_peak]
                       if settings[:stepVariable] == "v in m/s"
                           currentStepSize = CS[:v_peak] - drivingCourse[end-1][:v]
                       else
                           currentStepSize = settings[:stepSize] / 10.0^cycle
                       end
                   elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                       trainAtEnd = true
                       break

                   elseif drivingCourse[end][:v] == CS[:v_exit]
                       targetSpeedReached = true
                       break

                   elseif drivingCourse[end][:s] == nextPointOfInterest
                       break

                   else
                       # TODO: not needed. just for testing
                       error("ERROR at coasting until braking section: With the step variable ",settings[:stepVariable]," the while loop will be left although v<v_peak and s+s_braking<s_exit in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                   end
                   # delete last data point for recalculating the last step with reduced step size
                   pop!(drivingCourse)
                   pop!(BS[:dataPoints])
                   brakingStartReached = false

               else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        println("INFO: The train stops during the coasting section in CS",CS[:id],"                                                                ",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N and s_braking=",s_braking,"m.")

                   elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                       brakingStartReached = true
                       # delete last data point because it went to far
                       pop!(drivingCourse)
                       pop!(BS[:dataPoints])

                   elseif drivingCourse[end][:v] > CS[:v_peak] # if the train gets to fast it has to brake  # TODO: if accelereation and coasting functions will be combined this case is different for coasting and also the order of if cases is different
                       targetSpeedReached = true
                       # delete last data point because it went to far
                       pop!(drivingCourse)
                       pop!(BS[:dataPoints])

                       # while coasting the train brakes to hold v_peak (only one data point in the end of coasting is calculated like cruising at v_peak)
                       drivingCourse[end][:a] = 0.0
                       s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])

                       # recalculate s, t, v, E
                       s_constantCoasting = min(currentStepSize, CS[:s_exit] - (drivingCourse[end-1][:s] + s_braking))          # TODO: if settings[:stepVariable]=="s in m"
                       push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], s_constantCoasting, CS[:id]))
                       drivingCourse[end][:behavior] = BS[:type]
                       push!(BS[:dataPoints], drivingCourse[end][:i])

                   elseif drivingCourse[end][:s] > nextPointOfInterest
                       drivingCourse[end][:s] = nextPointOfInterest # round s down to nextPointOfInterest
                       drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]
                   else
                       if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                           brakingStartReached = true
                       end
                   end
               end
           end #for
       end #while

       # calculate the accumulated coasting section information
       merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                       :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                       :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                       :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                       :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

       CS[:t] = CS[:t] + BS[:t]          # total running time (in s)
       CS[:E] = CS[:E] + BS[:E]          # total energy consumption (in Ws)

       merge!(CS[:behaviorSections], Dict(:coasting=>BS))
   end

   return (CS, drivingCourse, brakingStartReached)
end #function addCoastingSection!


## This function calculates the data points of the braking section.
#    Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    # conditions for braking section
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]

    # use the conditions for the braking section
    if !targetSpeedReached && !trainAtEnd
        BS = createBehaviorSection("braking", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        while !targetSpeedReached && !trainAtEnd
            currentStepSize = settings[:stepSize]  # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while drivingCourse[end][:v] > CS[:v_exit] && !targetSpeedReached && drivingCourse[end][:s] < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest
                  # traction effort and resisting forces (in N):
                  calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

                  # acceleration (in m/s^2):
                  drivingCourse[end][:a] = train[:a_braking]
                  # TODO or: drivingCourse[end][:a] = calcBrakingAcceleration(drivingCourse[end][:v], CS[:v_exit], CS[:s_exit]-drivingCourse[end][:s])

                  if settings[:stepVariable] == "s in m" && ((drivingCourse[end][:v]/drivingCourse[end][:a])^2+2*currentStepSize/drivingCourse[end][:a])<0.0 || (drivingCourse[end][:v]^2+2*currentStepSize*drivingCourse[end][:a])<0.0
                      # create empty data point and set it for the values of s_exit and v_exit
                      push!(drivingCourse, createDataPoint())
                      drivingCourse[end][:i] = drivingCourse[end-1][:i]+1
                      drivingCourse[end][:behavior] = BS[:type]
                      push!(BS[:dataPoints], drivingCourse[end][:i])
                      recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                  else
                      # create the next data point
                      push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                      drivingCourse[end][:behavior] = BS[:type]
                      push!(BS[:dataPoints], drivingCourse[end][:i])
                  end
                  #println(drivingCourse[end][:i],".  s=",drivingCourse[end][:s],"  s_exit=", CS[:s_exit],"  v_exit=", CS[:v_exit],"  v=",drivingCourse[end][:v])
                end # while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                # TODO: is there a better way than rounding like in the following?
                if cycle < approximationLevel+1
                    if drivingCourse[end][:v] < CS[:v_exit]
                        if settings[:stepVariable] == "v in m/s"
                            currentStepSize = drivingCourse[end-1][:v] - CS[:v_exit]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end
                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        if settings[:stepVariable] == "s in m"
                            currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end
                    elseif drivingCourse[end][:v] == CS[:v_exit] && drivingCourse[end][:s] == CS[:s_exit]
                        targetSpeedReached = true
                        trainAtEnd = true
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit]
                        targetSpeedReached = true
                        trainAtEnd = true
                    #    println("during braking section in CS",CS[:id],": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        targetSpeedReached = true
                        trainAtEnd = true
                    #    println("during braking section in CS",CS[:id],": rounding v down from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                    elseif drivingCourse[end][:s] == nextPointOfInterest
                        break
                    end

                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] < 0.0
                        # reset last point with setting v=v_exit
                       targetSpeedReached = true
                       trainAtEnd = true
                    #   println("during braking section in CS",CS[:id],": rounding v up from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                       recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], 0.0)
                   #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                       break
                   elseif drivingCourse[end][:s] > CS[:s_exit]
                       targetSpeedReached = true
                       trainAtEnd = true
                #       println("during braking section in CS",CS[:id],": rounding s down from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                      # recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                       drivingCourse[end][:s] = CS[:s_exit]
                       break
                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        drivingCourse[end][:s] = nextPointOfInterest # round s down to nextPointOfInterest
                        drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit] && drivingCourse[end][:s] == CS[:s_exit]
                        targetSpeedReached = true
                        trainAtEnd = true
                        break
                    elseif drivingCourse[end][:v] < CS[:v_exit]
                        # reset last point with setting v=v_exit
                        targetSpeedReached = true
                        trainAtEnd = true
                    #    println("during braking section in CS",CS[:id],": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                   elseif drivingCourse[end][:v] == CS[:v_exit]
                        targetSpeedReached = true
                        trainAtEnd = true
                    #    println("during braking section in CS",CS[:id],": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        targetSpeedReached = true
                        trainAtEnd = true
                    #    println("during braking section in CS",CS[:id],": rounding v down from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                    else
                        # do nothing for example for drivingCourse[end][:s]==nextPointOfInterest
                    end
                end
            end #for
        end #while

       # calculate the accumulated coasting section information
       merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                       :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                       :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                       :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                       :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

       CS[:t] = CS[:t] + BS[:t]       # total running time (in s)
       CS[:E] = CS[:E] + BS[:E]       # total energy consumption (in Ws)

       merge!(CS[:behaviorSections], Dict(:braking=>BS))
    end  # else: return the characteristic section without a braking section

    return (CS, drivingCourse)
end #function addBrakingSection!


## This function calculates the data point of the standstill.
#    Therefore it gets its first data point and the characteristic section and returns the characteristic section including the standstill if needed.
function addStandstill!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    if drivingCourse[end][:v] == 0.0
        BS = createBehaviorSection("standstill", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        merge!(BS, Dict(:length => 0.0,                      # total length  (in m)
                        :t => 0.0,                           # total running time (in s)
                        :E => 0.0,                           # total energy consumption (in Ws)
                        :s_exit => drivingCourse[end][:s],    # last position  (in m)
                        :v_exit => drivingCourse[end][:v]))   # exit speed (in m/s)))
        drivingCourse[end][:behavior] = BS[:type]

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

        merge!(CS[:behaviorSections], Dict(:standstill => BS))
    end  # else: return the characteristic section without a standstillSection section
    return (CS, drivingCourse)
end #function addStandstill!

function mergeBehaviorSection!(BSs::Dict, BS::Dict)
    if !haskey(BSs, Symbol(BS[:type]))
        merge!(BSs, Dict(Symbol(BS[:type]) => BS))
    else
        number = "2"
        while haskey(BSs, Symbol(BS[:type]*number))
            number = string(parse(Int, number)+1)
        end
        merge!(BSs, Dict(Symbol(BS[:type]*number) => BS))
        println("INFO: The ",number,". ",BS[:type]," section has been created.      ! ! ! ! ! ! ! ! !")
    end
    return BSs
end #function mergeBehaviorSection!

function createBehaviorSection(type::String, s_entry::Real, v_entry::Real, startingPoint::Integer)
    BS= Dict(#:type => behavior,               # type of behavior section: breakFree, clearing, accelerating, cruising, diminishing, coasting, braking or standstill
            :type => type,                   # type of behavior section: "breakFree", "clearing", "accelerating", "cruising", "diminishing", "coasting", "braking" or "standstill"
            :length => 0.0,                 # total length  (in m)
            :s_entry => s_entry,            # first position (in m)
            :s_exit => 0.0,                 # last position  (in m)
            :t => 0.0,                      # total running time (in s)
            :E => 0.0,                      # total energy consumption (in Ws)
            :v_entry => v_entry,            # entry speed (in m/s)
            :v_exit => 0.0,                 # exit speed (in m/s)
            :dataPoints => [startingPoint]) # list of identifiers of the containing data points starting with the initial point
    return BS
end #function createBehaviorSection

"""
a data point is the smallest element of the driving course. One step of the step approach is between two data points
"""
function createDataPoint()
    dataPoint = Dict(:i => 0,                # identifier and counter variable of the dricing course
                    :behavior => "",         # type of behavior section the data point is part of ("breakFree", "clearing", "accelerating", "cruising", "diminishing", "coasting", "braking" or "standstill")
                                             # a data point which is the last point of one behavior section and the first point of the next behavior section will be attached to the latter
                    :s => 0.0,               # position (in m)
                    :Δs => 0.0,              # step size (in m)
                    :t => 0.0,               # point in time (in s)
                    :Δt => 0.0,              # step size (in s)
                    :v => 0.0,               # velocity (in m/s)
                    :Δv => 0.0,              # step size (in m/s)
                    :a => 0.0,               # acceleration (in m/s^2)
                    :W => 0.0,               # mechanical work (in Ws)
                    :ΔW => 0.0,              # mechanical work in this step (in Ws)
                    :E => 0.0,               # energy consumption (in Ws)
                    :ΔE => 0.0,              # energy consumption in this step (in Ws)
                    :F_T => 0.0,             # tractive effort (in N)
                    :F_R => 0.0,             # resisting force (in N)
                    :R_path => 0.0,          # path resistance (in N)
                    :R_train => 0.0,         # train resistance (in N)
                    :R_traction => 0.0,      # traction unit resistance (in N)
                    :R_wagons => 0.0)        # set of wagons resistance (in N)
    return dataPoint
end #function createDataPoint

function recalculateLastBrakingPoint!(drivingCourse, s_target, v_target)
    currentPoint = drivingCourse[end]
    previousPoint = drivingCourse[end-1]
   # set s and v
   currentPoint[:s] = s_target                                          # position (in m)
   currentPoint[:v] = v_target                                          # velocity (in m/s)
   currentPoint[:Δs] = currentPoint[:s] - previousPoint[:s]             # step size (in m)
   currentPoint[:Δv] = currentPoint[:v] - previousPoint[:v]             # step size (in m/s)

   # calculate other values
   previousPoint[:a] = calcBrakingAcceleration(previousPoint[:v], currentPoint[:v], currentPoint[:Δs])
#    # TODO: just for testing
#    if previousPoint[:a]<train[:a_braking] || previousPoint[:a]>=0.0
#       println("Warning: a_braking gets to high in CS ",CS[:id], "   with a=",previousPoint[:a]  ,"  >  ",train[:a_braking])
#    end
   currentPoint[:Δt] = calc_Δt_with_Δv(currentPoint[:Δv], previousPoint[:a])    # step size (in s)
   currentPoint[:t] = previousPoint[:t] + currentPoint[:Δt]                     # point in time (in s)

   currentPoint[:ΔW] = 0.0                                                      # mechanical work in this step (in Ws)
   currentPoint[:W] = previousPoint[:W] + currentPoint[:ΔW]                     # mechanical work (in Ws)
   currentPoint[:ΔE] = currentPoint[:ΔW]                                        # energy consumption in this step (in Ws)
   currentPoint[:E] = previousPoint[:E] + currentPoint[:ΔE]                     # energy consumption (in Ws)
end #function recalculateLastBrakingPoint

end #module Behavior
