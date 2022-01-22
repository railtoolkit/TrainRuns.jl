module Behavior

include("./DrivingDynamics.jl")
using .DrivingDynamics

export addAccelerationSection!, addAccelerationSectionUntilBraking!, addCruisingSection!, addCoastingSectionUntilBraking!, addBrakingSection!, addBrakingSectionStepwise!, addStandstill!,
# addBrakingSectionStepwise! is not used in the current version of the tool
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
    if bsType == "acceleration" || bsType == "diminishing"
        dataPoint[:F_T] = calculateTractiveEffort(dataPoint[:v], train[:tractiveEffortVelocityPairs])
    elseif bsType == "cruising"
        dataPoint[:F_T] = min(max(0.0, dataPoint[:F_R]), calculateTractiveEffort(dataPoint[:v], train[:tractiveEffortVelocityPairs]))
    else
        dataPoint[:F_T] = 0.0
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

    # creating the next data point
    newPoint = createDataPoint()
    newPoint[:i] = previousPoint[:i]+1         # identifier

    # calculate s, t, v, E
    if previousPoint[:a] == 0.0 # TODO: or better stepVariable=="s_cruising in m" ?
       newPoint[:Δs] = stepSize                                                     # step size (in m)
       newPoint[:Δt] = calc_Δt_with_constant_v(newPoint[:Δs], previousPoint[:v])    # step size (in s)
       newPoint[:Δv] = 0.0                                                          # step size (in m/s)
    elseif stepVariable == "s in m"                                             # distance step method
        newPoint[:Δs] = stepSize                                                    # step size (in m)
        if previousPoint[:a] < 0.0
            if ((previousPoint[:v]/previousPoint[:a])^2+2*newPoint[:Δs]/previousPoint[:a])<0.0 || (previousPoint[:v]^2+2*newPoint[:Δs]*previousPoint[:a])<0.0  # checking if the parts of the following square roots will be <0.0
                error("ERROR: The train stops during the acceleration section in CS",csId," because the tractive effort is lower than the resistant forces.",
                "       Before the stop the last point has the values s=",previousPoint[:s]," m,  v=",previousPoint[:v]," m/s,  a=",previousPoint[:a]," m/s^2,",
                "       F_T=",previousPoint[:F_T]," N,  R_traction=",previousPoint[:R_traction]," N,  R_wagons=",previousPoint[:R_wagons]," N,  R_path=",previousPoint[:R_path]," N.")
            end
        end
        newPoint[:Δt] = calc_Δt_with_Δs(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in s)
        newPoint[:Δv] = calc_Δv_with_Δs(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in m/s)
    elseif stepVariable =="t in s"                                                              # time step method
        newPoint[:Δt] =stepSize                                                                     # step size (in s)
        newPoint[:Δs] = calc_Δs_with_Δt(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in m)
        newPoint[:Δv] = calc_Δv_with_Δt(newPoint[:Δs], previousPoint[:a])                           # step size (in m/s)
    elseif stepVariable =="v in m/s"                                                            # velocity step method
        newPoint[:Δv] = stepSize * sign(previousPoint[:a])                                          # step size (in m/s)
        newPoint[:Δs] = calc_Δs_with_Δv(newPoint[:Δs], previousPoint[:a], previousPoint[:v])        # step size (in m)
        newPoint[:Δt] = calc_Δt_with_Δv(newPoint[:Δs], previousPoint[:a])                           # step size (in s)
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
# if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
"""
function detectFormerSpeedLimits(CSs::Vector{Dict}, csWithTrainHeadId::Integer, currentPoint::Dict, trainLength::Real)
    formerSpeedLimits=[]
    if csWithTrainHeadId > 1 && currentPoint[:s] - trainLength < CSs[csWithTrainHeadId][:s_entry]
        formerCsId=csWithTrainHeadId-1
        while formerCsId > 0 && currentPoint[:s] - trainLength < CSs[formerCsId][:s_exit]
            if CSs[formerCsId][:v_limit] < CSs[csWithTrainHeadId][:v_limit]    # TODO: is the position of trains tail < movingSection[:s_entry], v_limit of the first CS is used
                push!(formerSpeedLimits, [CSs[formerCsId][:s_exit], CSs[formerCsId][:v_limit]])
                for i in 1:length(formerSpeedLimits)-1
                    if formerSpeedLimits[i][2]<=formerSpeedLimits[end][2]
                        pop!(formerSpeedLimits)
                        break
                    end
                end
            end
            formerCsId=formerCsId-1
        end
    end
    return formerSpeedLimits
end # function detectFormerSpeedLimits

function considerFormerSpeedLimits!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}, formerSpeedLimits, accelerationSection::Dict)
 #  TODO: What is the type of formerSpeedLimits?   function considerFormerSpeedLimits!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}, formerSpeedLimits::Array{Array{AbstractFloat,1},1}, accelerationSection::Dict)
 #        would work: function considerFormerSpeedLimits!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}, formerSpeedLimits::Array{Any,1}, accelerationSection::Dict)
    if length(formerSpeedLimits) > 0
        # if a former speed limit has been exceeded the acceleration steps of this CS will be removed and a clearing section will be inserted before acceleration
        if drivingCourse[end][:v] > formerSpeedLimits[end][2]
            while drivingCourse[end][:s] > get(CS[:behaviorSections], :clearing, accelerationSection)[:s_entry]
                pop!(drivingCourse)
            end

            if haskey(CS[:behaviorSections], :clearing)
                CS[:t] = CS[:t]-CS[:behaviorSections][:clearing][:t]              # reducing the total running time (in s)
                CS[:E] = CS[:E]-CS[:behaviorSections][:clearing][:E]              # reducing the total energy consumption (in Ws)
                delete!(CS[:behaviorSections], :clearing)
            end

            # create a (new and longer) clearing section
            s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
            s_clearing = min(CS[:s_exit]-drivingCourse[end][:s]-s_braking, formerSpeedLimits[end][1]-(drivingCourse[end][:s]-train[:length]))

            if s_clearing > 0.0
                (CS, drivingCourse) = addCruisingSection!(CS, drivingCourse, s_clearing, settings, train, CSs, "clearing")
            else
                error("ERROR: clearing <=0.0 although it has to be >0.0 in CS ",CS[:id])
            end

            # 09/22: if  drivingCourse[end][:s] < CS[:s_exit]
            if  drivingCourse[end][:s] < CS[:s_exit] - s_braking
                # reset the accelerationSection
                accelerationSection = createBehaviorSection("acceleration", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
            else
                return (CS, drivingCourse, formerSpeedLimits, accelerationSection, true)
            end
        end

        # remove former speed limits of characteristic sections the train has left during the last step from the list
        while length(formerSpeedLimits) > 0 && drivingCourse[end][:s] - train[:length] >= formerSpeedLimits[end][1]
            pop!(formerSpeedLimits)
        end
    end
    return (CS, drivingCourse, formerSpeedLimits, accelerationSection, false)
end # function considerFormerSpeedLimits!


function getNextPointOfInterest(pointsOfInterest::Vector{Real}, s::Real)
    for POI in pointsOfInterest
        if POI > s
            return POI
        end
    end
    error("ERROR in getNextPointOfInterest: There is no POI ist higher than s.")
end #function getNextPointOfInterest

## This function calculates the data points of the breakFree section.
# Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for breakFree if needed.
# Info: currently the values of the breakFree section will be calculated like in the acceleration section
function addBreakFreeSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    if drivingCourse[end][:v]==0.0 && drivingCourse[end][:s]<CS[:s_exit]
        BS = createBehaviorSection("breakFree", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        # traction effort and resisting forces (in N):
        calculateForces!(drivingCourse[end], CSs, CS[:id], "acceleration", train, settings[:massModel])    # currently the tractive effort is calculated like in the acceleration section

        # acceleration (in m/s^2):
        drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])
        if drivingCourse[end][:a]<0.0
            error("ERROR: a<0 m/s^2 in the breakFree section !")
        elseif drivingCourse[end][:a] == 0.0
            error("ERROR: a=0 m/s^2 in the breakFree section !")
        end

        brokenFree = false
        calculationCycle = 1
        nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
        while !brokenFree
            # create the next data point
            #TODO: instead of the while loop this two line would also be enough for stepVariale s: stepSize = min(settings[:stepSize], getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])-BS[:s_entry])
            #push!(drivingCourse, moveAStep(drivingCourse[end], "s in m", stepSize, CS[:id]))
            push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], settings[:stepSize], CS[:id]))
            drivingCourse[end][:behavior] = BS[:type]
            push!(BS[:dataPoints], drivingCourse[end][:i])

            # check if a point of interest was missed
            if drivingCourse[end][:s] > nextPointOfInterest
                if settings[:stepVariable] == "s in m"
                    currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                else
                    currentStepSize = settings[:stepSize] / 10.0^cycle
                end

                if calculationCycle <= approximationLevel
                    calculationCycle = calculationCycle +1
                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])
                else
                    drivingCourse[end][:s] = nextPointOfInterest    # rounding s down to nextPointOfInterest
                    brokenFree = true
                    break#free
                end
            else
                brokenFree = true
                break#free
            end #if
        end #while

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


## This function calculates the data points of the acceleration section.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the acceleration section
function addAccelerationSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
 #=if drivingCourse would also be part of movingSectiong: function addAccelerationSection!(movingSection::Dict, csId::Integer, settings::Dict, train::Dict)
    CSs = movingSection[:characteristicSections]
    CS = CSs[csId]
    drivingCourse = movingSection[:drivingCourse]=#

    if drivingCourse[end][:v] == 0.0
        (CS, drivingCourse) = addBreakFreeSection!(CS, drivingCourse, settings, train, CSs)
    end #if

    calculateForces!(drivingCourse[end], CSs, CS[:id], "acceleration", train, settings[:massModel])
    if drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
        (CS, drivingCourse) = addDiminishingSection!(CS, drivingCourse, settings, train, CSs)
        calculateForces!(drivingCourse[end], CSs, CS[:id], "acceleration", train, settings[:massModel])
    end

    # if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
    formerSpeedLimits = detectFormerSpeedLimits(CSs, CS[:id], drivingCourse[end], train[:length])

    # conditions for acceleration section
    targetSpeedReached = drivingCourse[end][:v] >= CS[:v_peak]
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]

    # use the conditions for the acceleration section
    if !targetSpeedReached && !trainAtEnd && tractionSurplus
    #11/23 long version:    if drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:s] <CS[:s_exit] && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
        BS = createBehaviorSection("acceleration", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        while !targetSpeedReached && !trainAtEnd && tractionSurplus
            currentStepSize = settings[:stepSize]  # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                    # create the next data point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = BS[:type]
                    push!(BS[:dataPoints], drivingCourse[end][:i])

                    if length(formerSpeedLimits) > 0 # If the tail of the train is located in a former characteristic section with lower speed limit check if is is possible to accelerate as normal
                        (CS, drivingCourse, formerSpeedLimits, BS, endOfCsReached) = considerFormerSpeedLimits!(CS, drivingCourse, settings, train, CSs, formerSpeedLimits, BS)
                        if  endOfCsReached
                            return (CS, drivingCourse)
                        end #if
                        currentStepSize = settings[:stepSize]  # initialize the step size that can be reduced near intersections
                    end #if

                    # traction effort and resisting forces (in N)
                    calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])
                end #while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                    if drivingCourse[end][:v] <= 0.0
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        if settings[:stepVariable] == "s in m"
                            currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > CS[:v_peak]
                        if settings[:stepVariable] == "v in m/s"
                            currentStepSize = CS[:v_peak]-drivingCourse[end-1][:v]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        trainAtEnd = true
                        break

                    elseif drivingCourse[end][:v] == CS[:v_peak]
                        targetSpeedReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest
                        break

                    else
                        error("ERROR at acceleration section: With the step variable ", settings[:stepVariable]," the while loop will be left although v<v_peak and s<s_exit in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])
                else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        # push!(BS[:dataPoints], drivingCourse[end][:i])
                        error("ERROR: The train stops during the acceleration section in CS",CS[:id]," because the tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    elseif drivingCourse[end][:v] > CS[:v_peak]
                        targetSpeedReached = true
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        drivingCourse[end][:s] = nextPointOfInterest    # rounding s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        tractionSurplus = false
                        (CS, drivingCourse) = addDiminishingSection!(CS, drivingCourse, settings, train, CSs)
                        calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

                    else

                    end #if

                    # TODO is it possible to put this into to the if-fork?
                    if drivingCourse[end][:s] == CS[:s_exit]
                        trainAtEnd = true
                    end
                end #if
            end #for
        end #while

        if length(BS[:dataPoints]) > 1 # it is possible that the acceleration starts at v_peak, accelerates a step, is to high and drops the last point. then there is only one data point which is not a section.
            # calculate the accumulated acceleration section information
            merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                            :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                            :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                            :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                            :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

            CS[:t] = CS[:t] + BS[:t]             # total running time (in s)
            CS[:E] = CS[:E] + BS[:E]             # total energy consumption (in Ws)

            # TODO: this warning schould not be needed. just for testing
            if CS[:v_peak] < drivingCourse[end][:v]
                println("WARNING, v is getting to high at the end of the acceleration section.   v=",drivingCourse[end][:v] ,"  > v_peak=",CS[:v_peak])
            end

            merge!(CS[:behaviorSections], Dict(:acceleration => BS))
        end
    end

    return (CS, drivingCourse)
end #function addAccelerationSection!


## This function calculates the data points of the acceleration section.
function addAccelerationSectionUntilBraking!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    if drivingCourse[end][:v] == 0.0
        (CS, drivingCourse) = addBreakFreeSection!(CS, drivingCourse, settings, train, CSs)
    end #if

    calculateForces!(drivingCourse[end], CSs, CS[:id], "acceleration", train, settings[:massModel])
    if drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
        (CS, drivingCourse) = addDiminishingSection!(CS, drivingCourse, settings, train, CSs)
        calculateForces!(drivingCourse[end], CSs, CS[:id], "acceleration", train, settings[:massModel])
    end

    # if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
    formerSpeedLimits = detectFormerSpeedLimits(CSs, CS[:id], drivingCourse[end], train[:length])

    # conditions for acceleration section
    targetSpeedReached = drivingCourse[end][:v] >= CS[:v_peak]
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]

    # use the conditions for the acceleration section
    if !targetSpeedReached && !trainAtEnd && tractionSurplus
        BS = createBehaviorSection("acceleration", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]
        brakingStartReached = false

        while !targetSpeedReached && !trainAtEnd && tractionSurplus && !brakingStartReached
            currentStepSize=settings[:stepSize]   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                while drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:s] +s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]      # as long as s_i + s_braking < s_CSexit

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                    # create the next data point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = BS[:type]
                    push!(BS[:dataPoints], drivingCourse[end][:i])

                    if length(formerSpeedLimits) > 0 # If the tail of the train is located in a former characteristic section with lower speed limit check if is is possible to accelerate as normal
                        (CS, drivingCourse, formerSpeedLimits, BS, endOfCsReached) = considerFormerSpeedLimits!(CS, drivingCourse, settings, train, CSs, formerSpeedLimits, BS)
                        if  endOfCsReached
                            return (CS, drivingCourse)
                        end
                    end
                    calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])
                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                end #while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                    if drivingCourse[end][:v] <= 0.0
                        currentStepSize = settings[:stepSize] / 10.0^cycle
                        # TODO 01/21 should not be needed anymore with diminishing.

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] +s_braking > CS[:s_exit]
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        if settings[:stepVariable] == "s in m"
                            currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > CS[:v_peak]
                        if settings[:stepVariable]=="v in m/s"
                            currentStepSize = CS[:v_peak]-drivingCourse[end-1][:v]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        trainAtEnd = true
                        break

                    elseif drivingCourse[end][:v] == CS[:v_peak]
                        targetSpeedReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest
                        break

                    else
                        error("ERROR at acceleration until braking section: With the step variable ",settings[:stepVariable]," the while loop will be left although v<v_peak and s<s_exit in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])

                else # if the level of approximation is reached
                    if drivingCourse[end][:v]<=0.0
                    # 01/21 should not be needed anymore with diminishing.
                        # push!(BS[:dataPoints], drivingCourse[end][:i])
                        error("ERROR: The train stops during the acceleration section in CS",CS[:id]," because the tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    elseif drivingCourse[end][:v] > CS[:v_peak]
                        targetSpeedReached = true
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        brakingStartReached = true
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        drivingCourse[end][:s] = nextPointOfInterest # rounding s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        tractionSurplus = false
                        (CS, drivingCourse) = addDiminishingSection!(CS, drivingCourse, settings, train, CSs)
                        calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

                    else

                    end

                    # TODO is it possible to put this into to the if-fork?
                    if drivingCourse[end][:s] == CS[:s_exit]
                        trainAtEnd = true
                    end
                end
            end #for
        end #while

        if length(BS[:dataPoints]) > 1 # TODO: is it still possible that it is <=1 although there is a separate diminishing section?
            # calculate the accumulated acceleration section information
            merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                            :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                            :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                            :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                            :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

            CS[:v_peak] = max(drivingCourse[end][:v], CS[:v_entry])      # setting v_peak to the last data points velocity which is the highest reachable value in this characteristic section or to v_entry in case it is higher when driving on a path with high resistances
            CS[:t] = CS[:t] + BS[:t]       # total running time (in s)
            CS[:E] = CS[:E] + BS[:E]       # total energy consumption (in Ws)

            merge!(CS[:behaviorSections], Dict(:acceleration=>BS))
        end
    end

    return (CS, drivingCourse)
end #function addAccelerationSectionUntilBraking!


## This function calculates the data points of the cruising section.
#   Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for cruising if needed.
function addCruisingSection!(CS::Dict, drivingCourse::Vector{Dict}, s_cruising::Real, settings::Dict, train::Dict, CSs::Vector{Dict}, cruisingType::String)
    # traction effort and resisting forces (in N)
    calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])

    if drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
        (CS, drivingCourse) = addDiminishingSection!(CS, drivingCourse, settings, train, CSs)
        # 01/08 old with DataPoint as struct: old drivingCourse[end] = DataPoint(calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel]))
        calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])
        s_cruising = max(0.0, s_cruising-get(CS[:behaviorSections], :diminishing, Dict(:length=>0.0))[:length])
    end
    if drivingCourse[end][:v]>0.0 && drivingCourse[end][:v]<=CS[:v_peak] && drivingCourse[end][:s]<CS[:s_exit] && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
    # 11/22 old: if drivingCourse[end][:v]>0.0 && drivingCourse[end][:v]<=CS[:v_peak] && drivingCourse[end][:s]<CS[:s_exit]
        BS = createBehaviorSection(cruisingType, drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]

        # TODO: necessary?
        s_cruising = min(s_cruising, CS[:s_exit]-BS[:s_entry])

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel]) # TODO: or give BS[:type] instead of "cruising"?
        # 11/05 old: drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel]))

        if settings[:massModel]=="homogeneous strip" && CS[:id] > 1
            # conditions for cruising section
            trainInPreviousCS = drivingCourse[end][:s] < CS[:s_entry] + train[:length]
            trainAtEnd = drivingCourse[end][:s] >= BS[:s_entry] +s_cruising
            tractionSurplus = drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]

            # use the conditions for the cruising section
            while trainInPreviousCS && !trainAtEnd && tractionSurplus
                currentStepSize = settings[:stepSize]
                nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            # better?    nextPointOfInterest = min(BS[:s_entry]+s_cruising, getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s]))

                for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                    while drivingCourse[end][:s] < CS[:s_entry] + train[:length] && drivingCourse[end][:s] < BS[:s_entry] +s_cruising && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:F_T]>=drivingCourse[end][:F_R]
                        # TODO: whithout CSs should work as well, no? while drivingCourse[end][:s] < CSs[CS[:id]][:s_entry] + train[:length] && drivingCourse[end][:s]<BS[:s_entry]+s_cruising && drivingCourse[end][:F_T]>=drivingCourse[end][:F_R] #&& drivingCourse[end][:v]<=CS[:v_peak] && drivingCourse[end][:s]<CS[:s_exit]
                     # the tractive effort is lower than the resisiting forces and the train has use the highest possible effort to try to stay at v_peak OR the mass model homogeneous strip is used and parts of the train are still in former CS
                     #TODO: maybe just consider former CS with different path resistance?
                     #TODO: what about the case: After leaving a former CS with steep gradient the train can accelerate. Now in this tool the train will cruise at v_i. Just accelerating until v_peak could make problems for energy saving by shortening the acceleration section

                        # acceleration (in m/s^2):
                        drivingCourse[end][:a] = 0.0

                        # create the next data point
                        if settings[:stepVariable]=="s in m"
                            push!(drivingCourse, moveAStep(drivingCourse[end], "s_cruising in m", currentStepSize, CS[:id]))
                        else
                            push!(drivingCourse, moveAStep(drivingCourse[end], "s_cruising in m", train[:length]/(10.0^cycle), CS[:id])) # TODO which step size should be used?
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

                        else # TODO copied from addAccelerationSection -> probably not needed here !?
                            error("ERROR at cruising section: With the step variable ",settings[:stepVariable]," the while loop will be left although the if cases don't apply in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                        end

                        # delete last data point for recalculating the last step with reduced step size
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    else # if the level of approximation is reached
                        if drivingCourse[end][:s] > nextPointOfInterest
                            drivingCourse[end][:s] = nextPointOfInterest # rounding s down to nextPointOfInterest
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
                            (CS, drivingCourse) = addDiminishingSection!(CS, drivingCourse, settings, train, CSs)
                            calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])

                        #    s_cruising=max(0.0, s_cruising-get(CS[:behaviorSections], :diminishing, Dict(length=>0.0))[:length])

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
            s_cruisingRemaining = nextPointOfInterest - drivingCourse[end][:s]

            # create the next data point
            push!(drivingCourse, moveAStep(drivingCourse[end], "s_cruising in m", s_cruisingRemaining, CS[:id]))
            drivingCourse[end][:behavior] = BS[:type]
            push!(BS[:dataPoints], drivingCourse[end][:i])

            calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings[:massModel])
        end #while

        # calculate the accumulated cruising section information
        merge!(BS, Dict(:length => drivingCourse[end][:s] - BS[:s_entry],                         # total length  (in m)
                        :s_exit => drivingCourse[end][:s],                                        # last position  (in m)
                        :t => drivingCourse[end][:t] - drivingCourse[BS[:dataPoints][1]][:t],     # total running time (in s)
                        :E => drivingCourse[end][:E] - drivingCourse[BS[:dataPoints][1]][:E],     # total energy consumption (in Ws)
                        :v_exit => drivingCourse[end][:v]))                                       # exit speed (in m/s)))

        CS[:t] = CS[:t] + BS[:t]       # total running time (in s)
        CS[:E] = CS[:E] + BS[:E]       # total energy consumption (in Ws)

        merge!(CS[:behaviorSections], Dict(Symbol(BS[:type]) => BS))
    end # else: return the characteristic section without a cruising section

    return (CS, drivingCourse)
end #function addCruisingSection!


## This function calculates the data points for diminishing run when using maximum tractive effort and still getting slower
function addDiminishingSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    calculateForces!(drivingCourse[end], CSs, CS[:id], "diminishing", train, settings[:massModel])
    # conditions for diminishing section
    targetSpeedReached = drivingCourse[end][:v] <= 0.0
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]

    # use the conditions for the diminishing section
    if !tractionSurplus && !targetSpeedReached && !trainAtEnd
        BS = createBehaviorSection("diminishing", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        drivingCourse[end][:behavior] = BS[:type]
        brakingStartReached = false

        while !tractionSurplus && !targetSpeedReached && !trainAtEnd && !brakingStartReached
            currentStepSize=settings[:stepSize]   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                while drivingCourse[end][:F_T] <= drivingCourse[end][:F_R] && drivingCourse[end][:s] + s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest && drivingCourse[end][:v]>0.0       # as long as s_i + s_braking < s_end
                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                    # create the next data point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = BS[:type]
                    push!(BS[:dataPoints], drivingCourse[end][:i])

                    s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                    calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])
                end #while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                    if drivingCourse[end][:v] <= 0.0
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        if settings[:stepVariable] == "s in m"
                            currentStepSize = nextPointOfInterest - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        brakingStartReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest
                        break

                    else
                        error("ERROR during diminishing run: With the step variable ",settings[:stepVariable]," the while loop will be left although s+s_braking<s_exit && v>0.0  in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(BS[:dataPoints])

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        targetSpeedReached = true
                        # push!(BS[:dataPoints], drivingCourse[end][:i])
                        error("ERROR: The train stops during diminishing run in CS",CS[:id]," because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        brakingStartReached = true
                        pop!(drivingCourse)
                        pop!(BS[:dataPoints])

                    elseif drivingCourse[end][:s] > nextPointOfInterest
                        drivingCourse[end][:s] = nextPointOfInterest # rounding s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                        tractionSurplus = true
                        break

                    else

                    end #if
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

            merge!(CS[:behaviorSections], Dict(:diminishing=>BS))
        end
    end

    return (CS, drivingCourse)
end #function addDiminishingSection!


## This function calculates the data points of the coasting section.
# Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the coasting section
function addCoastingSectionUntilBraking!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict})
    ## if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
    #formerSpeedLimits = detectFormerSpeedLimits(CSs, CS[:id], drivingCourse[end], train[:length])

    # conditions for coasting section
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
    trainAtEnd = drivingCourse[end][:s] >= CS[:s_exit]

    # use the conditions for the coasting section
    if !targetSpeedReached && !trainAtEnd
       BS = createBehaviorSection("coasting", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
       drivingCourse[end][:behavior] = BS[:type]
       brakingStartReached = false

       while !targetSpeedReached && !trainAtEnd && !brakingStartReached
           currentStepSize=settings[:stepSize]  # initialize the step size that can be reduced near intersections
           nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])

           for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
                while drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:v] <= CS[:v_peak] && drivingCourse[end][:s] + s_braking < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest # as long as s_i + s_braking < s_end
                   # traction effort and resisting forces (in N):
                   calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

                   # acceleration (in m/s^2):
                   drivingCourse[end][:a] = calcAcceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train[:m_train], train[:ξ_train])

                   # creating the next data point
                   push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
                   drivingCourse[end][:behavior] = BS[:type]
                   push!(BS[:dataPoints], drivingCourse[end][:i])

                   s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
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
                        currentStepSize = settings[:stepSize] / 10.0^cycle

                   elseif drivingCourse[end][:v] > CS[:v_peak]
                       if settings[:stepVariable]=="v in m/s"
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

               else # if the level of approximation is reached
                    if drivingCourse[end][:v]<=0.0
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
                       drivingCourse[end][:s] = nextPointOfInterest # rounding s down to nextPointOfInterest
                   else

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

   return (CS, drivingCourse)
end #function addCoastingSectionUntilBraking!


## This function calculates the data points of the braking section. (standard braking section with only two data points)
#    Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingSection!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}) #, s_braking::AbstractFloat)
    # function addBrakingSection!(CS::Dict, drivingCourse::Vector{Dict}, massModel::String, train::Dict, CSs::Vector{Dict}) #, s_braking::AbstractFloat)
    if drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:s] < CS[:s_exit]
        BS = createBehaviorSection("braking", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
        BS[:s_exit] = CS[:s_exit]        # last position (in m)
        drivingCourse[end][:behavior] = BS[:type]


        while drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:s] < BS[:s_exit]
            nextPointOfInterest = min(BS[:s_exit], getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s]))

            if nextPointOfInterest < BS[:s_exit]
                # traction effort and resisting forces (in N)
                calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])
                drivingCourse[end][:a] = calcBrakingAcceleration(drivingCourse[end][:v], CS[:v_exit], BS[:s_exit]-drivingCourse[end][:s])
                # TODO: or just take train[:a_braking]? difference ist by 0.0000000001 m/s^2:   drivingCourse[end][:a] = train[:a_braking]
                # println("a_braking till ",nextPointOfInterest,": ", calcBrakingAcceleration(drivingCourse[end][:v], CS[:v_exit], BS[:s_exit]-drivingCourse[end][:s]), ".  It should be: ",train[:a_braking])

                # calculate the braking distance to the next point of interest
                stepSize = nextPointOfInterest - drivingCourse[end][:s]

                # create the next data point
                push!(drivingCourse, moveAStep(drivingCourse[end], "s in m", stepSize, CS[:id]))
                drivingCourse[end][:behavior] = BS[:type]
                push!(BS[:dataPoints], drivingCourse[end][:i])

            else # so if nextPointOfInterest == BS[:s_exit]
                # traction effort and resisting forces (in N)
                calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

                push!(drivingCourse, createDataPoint())
                drivingCourse[end][:i] = drivingCourse[end-1][:i]+1                       # incrementing the number of the data point
                drivingCourse[end][:behavior] = BS[:type]
                push!(BS[:dataPoints], drivingCourse[end][:i])               # refering from the breaking section to the last of its data points

                # calculate s, t, v
                drivingCourse[end][:s] = BS[:s_exit]                                            # position (in m)
                drivingCourse[end][:v] = CS[:v_exit]                                            # velocity (in m/s)
                drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]     # step size (in m)
                drivingCourse[end][:Δv] = drivingCourse[end][:v] - drivingCourse[end-1][:v]     # step size (in m/s)

                drivingCourse[end-1][:a] = calcBrakingAcceleration(drivingCourse[end-1][:v], drivingCourse[end][:v], drivingCourse[end][:Δs])
                drivingCourse[end][:Δt] = calc_Δt_with_Δv(drivingCourse[end][:Δv], drivingCourse[end-1][:a])        # step size (in s)
                drivingCourse[end][:t] = drivingCourse[end-1][:t] + drivingCourse[end][:Δt]         # point in time (in s)

                drivingCourse[end][:ΔW] = 0.0                                                       # mechanical work in this step (in Ws)
                drivingCourse[end][:W] = drivingCourse[end-1][:W] + drivingCourse[end][:ΔW]         # mechanical work (in Ws)
                drivingCourse[end][:ΔE] = drivingCourse[end][:ΔW]                                   # energy consumption in this step (in Ws)
                drivingCourse[end][:E] = drivingCourse[end-1][:E] + drivingCourse[end][:ΔE]         # energy consumption (in Ws)

                break
            end #if
        end #while

        merge!(BS, Dict(:length => drivingCourse[end][:Δs],   # total length  (in m)
                        #:s_exit => drivingCourse[end][:s],    # last position  (in m)
                        :t => drivingCourse[end][:Δt],        # total running time (in s)
                        :E => drivingCourse[end][:ΔE],        # total energy consumption (in Ws)
                        :v_exit => drivingCourse[end][:v]))   # exit speed (in m/s)))

        CS[:t] = CS[:t] + BS[:t]       # total running time (in s)
        CS[:E] = CS[:E] + BS[:E]       # total energy consumption (in Ws)

        merge!(CS[:behaviorSections], Dict(:braking=>BS))
    end  # else: return the characteristic section without a braking section
    return (CS, drivingCourse)
end #function addBrakingSection!


## This function calculates the data points of the braking section. # 09/07 new braking section with more than two data points
#    Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingSectionStepwise!(CS::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, CSs::Vector{Dict}) #, s_braking::AbstractFloat)
    #= TODO from 2022/01/22: integrate points of interest
   if drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:s] < CS[:s_exit]
       BS = createBehaviorSection("braking", drivingCourse[end][:s], drivingCourse[end][:v], drivingCourse[end][:i])
       drivingCourse[end][:behavior] = BS[:type]

       currentStepSize=settings[:stepSize]  # initialize the step size that can be reduced near intersections
       velocityIsPositive = true
       while drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:s] < CS[:s_exit] && velocityIsPositive
          # traction effort and resisting forces (in N):
          calculateForces!(drivingCourse[end], CSs, CS[:id], BS[:type], train, settings[:massModel])

          # acceleration (in m/s^2):
          drivingCourse[end][:a] = train[:a_braking]

          # creating the next data point

          #TODO moveAStep should give back true or false for success or failure e.g. with dropping below v=0 m/s
           #at the moment it is only for stepVariable=="s in m"
          if settings[:stepVariable] == "s in m"
              if ((drivingCourse[end][:v]/drivingCourse[end][:a])^2+2*currentStepSize/drivingCourse[end][:a])<0.0 || (drivingCourse[end][:v]^2+2*currentStepSize*drivingCourse[end][:a])<0.0
                  velocityIsPositive=false
                  break
              end
          end
          push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS[:id]))
          drivingCourse[end][:behavior] = BS[:type]
          push!(BS[:dataPoints], drivingCourse[end][:i])

          # s_braking = calcBrakingDistance(drivingCourse[end][:v], CS[:v_exit], train[:a_braking])
       end # while

       if drivingCourse[end][:v] < CS[:v_exit] || !velocityIsPositive
           # calculate s, t, v
           drivingCourse[end][:s] = CS[:s_exit]                                         # position (in m)
           drivingCourse[end][:v] = CS[:v_exit]                                         # velocity (in m/s)
           drivingCourse[end][:Δs] = drivingCourse[end][:s] - drivingCourse[end-1][:s]  # step size (in m)
           drivingCourse[end][:Δv] = drivingCourse[end][:v] - drivingCourse[end-1][:v]  # step size (in m/s)

           #drivingCourse[end-1][:a]=round(calcBrakingAcceleration(drivingCourse[end-1][:v], drivingCourse[end][:v], drivingCourse[end][:Δs]), digits=approximationLevel)   # acceleration (in m/s^2) (rounding because it should not be less than a_braking)
           drivingCourse[end-1][:a] = calcBrakingAcceleration(drivingCourse[end-1][:v], drivingCourse[end][:v], drivingCourse[end][:Δs])
       #    if drivingCourse[end-1][:a]<train[:a_braking] || drivingCourse[end-1][:a]>=0.0
       #        println("Warning: a_braking gets to high in CS ",CS[:id], "   with a=",drivingCourse[end-1][:a]  ,"  >  ",train[:a_braking])
       #    end
           drivingCourse[end][:Δt] = calc_Δt_with_Δv(drivingCourse[end][:Δv], drivingCourse[end-1][:a])     # step size (in s)
           drivingCourse[end][:t] = drivingCourse[end-1][:t] + drivingCourse[end][:Δt]      # point in time (in s)

           drivingCourse[end][:ΔW] = 0.0                                                    # mechanical work in this step (in Ws)
           drivingCourse[end][:W] = drivingCourse[end-1][:W] + drivingCourse[end][:ΔW]      # mechanical work (in Ws)
           drivingCourse[end][:ΔE] = drivingCourse[end][:ΔW]                                # energy consumption in this step (in Ws)
           drivingCourse[end][:E] = drivingCourse[end-1][:E] + drivingCourse[end][:ΔE]      # energy consumption (in Ws)
       elseif drivingCourse[end][:s] > CS[:s_exit]
          error("At the end of braking: s>s_exit but v>v_exit")
       else

       end

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
   =#
   return (CS, drivingCourse)
end #function addBrakingSectionStepwise!


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

function createBehaviorSection(type::String, s_entry::Real, v_entry::Real, startingPoint::Integer)
    BS= Dict(#:type => behavior,               # type of behavior section: breakFree, clearing, acceleration, cruising, diminishing, coasting, braking or standstill
            :type => type,                   # type of behavior section: "breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "braking" or "standstill"
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
                    :behavior => "",         # type of behavior section the data point is part of ("breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "braking" or "standstill")
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

end #module Behavior
