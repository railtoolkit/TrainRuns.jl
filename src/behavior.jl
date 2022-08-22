#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

## This function calculates the support points of the breakFree section.
# Therefore it gets its first support point and the characteristic section and returns the characteristic section including the behavior section for breakFree if needed.
# Info: currently the values of the breakFree section will be calculated like in the accelerating section
function addBreakFreeSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    CS = CSs[csId]

    # conditions for the break free section
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    trainIsHalting = drivingCourse[end][:v] == 0.0

    if trainIsHalting && !endOfCSReached
        drivingMode = "breakFree"
        drivingCourse[end][:behavior] = drivingMode
        startingPoint = length(drivingCourse)

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, csId, "accelerating", train, settings.massModel)    # currently the tractive effort is calculated like in the accelerating section

        # calculate the breakFree section with calculating the accelerating section and just using the first step and removing the rest
        try (drivingCourse, stateFlags) = addAcceleratingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)
        catch(acceleratingError)
            println("This error happened during the break free phase that is using the accelerating function:")
            rethrow(acceleratingError)
        end

        # delete every supportPoint except the first two
        while length(drivingCourse) > startingPoint +1
            pop!(drivingCourse)
        end

        # change the accelerating data to break free
        drivingCourse[end-1][:behavior] = drivingMode
        drivingCourse[end][:behavior] = drivingMode
    end # else: return the characteristic section without a breakFree section

    # determine state flags
    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

    stateFlags[:endOfCSReached] = drivingCourse[end][:s] >= CS[:s_exit]
    stateFlags[:brakingStartReached] = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
    stateFlags[:tractionDeficit] = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]   # or add another flag for equal forces?
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = false
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_limit]
    stateFlags[:error] = drivingCourse[end][:v] > CS[:v_limit] || drivingCourse[end][:s] > CS[:s_exit]

    return (drivingCourse, stateFlags)
end #function addBreakFreeSection!

## This function calculates the support points of the clearing section.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the clearing section.
function addClearingSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    CS = CSs[csId]

    if stateFlags[:previousSpeedLimitReached]
        lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

        s_clearing = min(CS[:s_exit]-drivingCourse[end][:s]-s_braking, lowestSpeedLimit[:s_end] - drivingCourse[end][:s])
        if s_clearing > 0.0
            (drivingCourse, stateFlags) = addCruisingSection!(drivingCourse, stateFlags, CSs, csId, settings, train, "clearing",  s_clearing)
            calculateForces!(drivingCourse[end], CSs, csId, "accelerating", train, settings.massModel)
        else
            error("ERROR: clearing <=0.0 although it has to be >0.0 in CS ",csId)
        end
        #stateFlags[:previousSpeedLimitReached] = false
        lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
        stateFlags[:previousSpeedLimitReached] = lowestSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= lowestSpeedLimit[:v]
    else
        stateFlags[:error] = true
    end

    return (drivingCourse, stateFlags)
end #function addClearingSection

## This function calculates the support points of the accelerating section.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the accelerating section
function addAcceleratingSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    CS = CSs[csId]

    calculateForces!(drivingCourse[end], CSs, csId, "accelerating", train, settings.massModel)
    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

    # conditions for the accelerating section
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]
    previousSpeedLimitReached = stateFlags[:previousSpeedLimitReached]
    speedLimitReached = drivingCourse[end][:v] >= CS[:v_limit] || stateFlags[:speedLimitReached]

    # use the conditions for the accelerating section
    if !speedLimitReached && !endOfCSReached && tractionSurplus && !brakingStartReached
        drivingMode = "accelerating"
        drivingCourse[end][:behavior] = drivingMode

        lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
        previousSpeedLimitReached = lowestSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= lowestSpeedLimit[:v]

        while !speedLimitReached && !endOfCSReached && tractionSurplus && !brakingStartReached && !previousSpeedLimitReached
            currentStepSize = settings.stepSize   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]

            for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !speedLimitReached && !brakingStartReached && !pointOfInterestReached && tractionSurplus && !previousSpeedLimitReached
                    if drivingCourse[end][:s] >= lowestSpeedLimit[:s_end]
                        # could be asked after creating an support point. This way here prevents even a minimal exceedance of speed limit. On the other hand the train cruises possibly a little to long
                        lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
                    end

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train.m_train_full, train.ξ_train)

                    # create the next support point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, csId))
                    drivingCourse[end][:behavior] = drivingMode

                    calculateForces!(drivingCourse[end], CSs, csId, drivingMode, train, settings.massModel)

                    # conditions for the next while cycle
                    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

                    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
                    speedLimitReached = drivingCourse[end][:v] >= CS[:v_limit]
                    previousSpeedLimitReached  = lowestSpeedLimit[:v] < CS[:v_limit] && (drivingCourse[end][:v] > lowestSpeedLimit[:v] || (drivingCourse[end][:v] == lowestSpeedLimit[:v] && drivingCourse[end][:s] < lowestSpeedLimit[:s_end]))
                    pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]   # POIs include s_exit as well
                    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                end #while

                if csId==0
                    testFlag = true
                else
                    testFlag = false     # for testing
                end

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel+1
                    if drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," <= F_R=",drivingCourse[end][:F_R])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPOI=",nextPointOfInterest[:s])    # for testing
                        if settings.stepVariable == :distance
                            currentStepSize = nextPointOfInterest[:s] - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > lowestSpeedLimit[:v]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_lowestLimit=", lowestSpeedLimit[:v])    # for testing
                        if settings.stepVariable == :velocity
                            currentStepSize = lowestSpeedLimit[:v] - drivingCourse[end-1][:v]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                        if s_braking == 0.0
                            endOfCSReached = true
                        end
                        break

                    elseif drivingCourse[end][:v] == lowestSpeedLimit[:v]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," == v_lowestLimit=", lowestSpeedLimit[:v])    # for testing
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest[:s]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," == nextPOI=",nextPointOfInterest[:s])    # for testing
                        if nextPointOfInterest[:s] == CS[:s_exit]
                            endOfCSReached = true
                        end
                        break

                    else
                        println("v=",drivingCourse[end][:v],"   v_limit= ", CS[:v_limit] , "  v_lowestLimit=", lowestSpeedLimit[:v])
                        println("s=" ,drivingCourse[end][:s],"   s_exit=", CS[:s_exit], "   s+s_braking=", drivingCourse[end][:s] +s_braking,"   nextPOI=",nextPointOfInterest[:s])
                        println("F_T=",drivingCourse[end][:F_T] ,"   F_R=", drivingCourse[end][:F_R])

                        error("ERROR at accelerating section: With the step variable ",settings.stepVariable," the while loop will be left although v<v_limit and s<s_exit in CS",csId,"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    brakingStartReached = false
                    previousSpeedLimitReached = false
                    speedLimitReached = false
                    endOfCSReached = false
                    pointOfInterestReached = false
                    tractionSurplus = true

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] > lowestSpeedLimit[:v]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_lowestLimit=", lowestSpeedLimit[:v], "with v_limit=",CS[:v_limit])    # for testing
                        pop!(drivingCourse)

                        # conditions for the next section
                        brakingStartReached = false

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        if s_braking > 0.0
                            pop!(drivingCourse)

                        else
                            drivingCourse[end][:s] = CS[:s_exit] # round s down to CS[:s_exit]
                        end

                    elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest[:s]=",nextPointOfInterest[:s])    # for testing
                        drivingCourse[end][:s] = nextPointOfInterest[:s] # round s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        testFlag && println("in CS",csId," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," <= F_R=",drivingCourse[end][:F_R])    # for testing

                    else
                        if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                            testFlag && println("in CS",csId," accelerating cycle",cycle," else case and there: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        elseif drivingCourse[end][:v] == lowestSpeedLimit[:v]
                            testFlag && println("in CS",csId," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," == v_lowestLimit=", lowestSpeedLimit[:v])    # for testing

                        end
                    end

                    # TODO is it possible to put this into to the if-fork?
                    if drivingCourse[end][:s] == CS[:s_exit]
                        endOfCSReached = true
                    end

                end
            end #for

            if drivingCourse[end][:s] == CS[:s_exit]
                endOfCSReached = true
            end
            if drivingCourse[end][:s] == nextPointOfInterest[:s]
                drivingCourse[end][:label] = nextPointOfInterest[:label]
            end

        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = !(tractionSurplus || drivingCourse[end][:F_T] == drivingCourse[end][:F_R])   # or add another flag for equal forces?
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = previousSpeedLimitReached
    stateFlags[:speedLimitReached] = speedLimitReached
    stateFlags[:error] = !(endOfCSReached || brakingStartReached || stateFlags[:tractionDeficit] || previousSpeedLimitReached || speedLimitReached)

    return (drivingCourse, stateFlags)
end #function addAcceleratingSection!


## This function calculates the support points of the cruising section.
#   Therefore it gets its first support point and the characteristic section and returns the characteristic section including the behavior section for cruising if needed.
function addCruisingSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train, cruisingType::String, s_cruising::Real)
    CS = CSs[csId]

    trainIsClearing = cruisingType == "clearing"
    trainIsBrakingDownhill = cruisingType == "downhillBraking"

    # traction effort and resisting forces (in N)
    if !trainIsBrakingDownhill  # TODO: or just give drivingMode instead of "cruising"/"braking"?
        calculateForces!(drivingCourse[end], CSs, csId, "cruising", train, settings.massModel)
    else
        calculateForces!(drivingCourse[end], CSs, csId, "braking", train, settings.massModel)
    end

    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

    # conditions for cruising section
    #s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]
    speedIsValid = drivingCourse[end][:v] > 0.0 && drivingCourse[end][:v] <= CS[:v_limit]
    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
    targetPositionReached = s_cruising == 0.0
    resistingForceNegative = drivingCourse[end][:F_R] < 0

    if speedIsValid && !brakingStartReached && !tractionDeficit && !targetPositionReached
        drivingMode = cruisingType
        drivingCourse[end][:behavior] = drivingMode
        # TODO: necessary?
        targetPosition = min(drivingCourse[end][:s] + s_cruising, CS[:s_exit])
        # 07/12 old: s_cruising = min(s_cruising, CS[:s_exit]-drivingCourse[end][:s])

        # traction effort and resisting forces (in N)
        if !trainIsBrakingDownhill
            calculateForces!(drivingCourse[end], CSs, csId, "cruising", train, settings.massModel)
        else
            calculateForces!(drivingCourse[end], CSs, csId, "braking", train, settings.massModel)
        end

        if settings.massModel == :homogeneous_strip && csId > 1
            # conditions for cruising section
            trainInPreviousCS = drivingCourse[end][:s] < CS[:s_entry] + train.length
            targetPositionReached = drivingCourse[end][:s] >= targetPosition
            resistingForceNegative = drivingCourse[end][:F_R] < 0.0
                # TODO: change? to: correctCruisingType = (trainIsClearing || (trainIsBrakingDownhill == drivingCourse[end][:F_R] < 0)) # while clearing tractive or braking force can be used

            # use the conditions for the cruising section
            while trainInPreviousCS && !targetPositionReached && !tractionDeficit && (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
                currentStepSize = settings.stepSize
                nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
                pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]

                for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                    while trainInPreviousCS && !targetPositionReached && !pointOfInterestReached && !tractionDeficit && (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
                     # the tractive effort is lower than the resisting forces and the train has to use the highest possible effort to try to stay at v_limit OR the mass model homogeneous strip is used and parts of the train are still in former CS
                      #TODO: maybe just consider former CS with different path resistance?
                        # tractive effort (in N):
                        if !trainIsBrakingDownhill
                            drivingCourse[end][:F_T] = min(drivingCourse[end][:F_T], max(0.0, drivingCourse[end][:F_R]))
                        else
                            drivingCourse[end][:F_T] = 0.0
                        end


                        # acceleration (in m/s^2):
                        drivingCourse[end][:a] = 0.0

                        # create the next support point
                        if settings.stepVariable == :distance || settings.stepVariable == time
                            push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, csId))
                        else
                            push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, train.length/(10.0^cycle), csId)) # TODO which step size should be used?
                        end
                        drivingCourse[end][:behavior] = drivingMode

                        # traction effort and resisting forces (in N)
                        calculateForces!(drivingCourse[end], CSs, csId, "default", train, settings.massModel)

                        # conditions for the next while cycle
                        pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]   # POIs include s_exit as well
                        tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                        targetPositionReached = drivingCourse[end][:s] >= targetPosition
                        trainInPreviousCS = drivingCourse[end][:s] < CS[:s_entry] + train.length
                        resistingForceNegative = drivingCourse[end][:F_R] < 0.0
                    end #while

                    # check which limit was reached and adjust the currentStepSize for the next cycle
                    if cycle < settings.approxLevel+1
                        if drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                            currentStepSize = settings.stepSize / 10.0^cycle

                        elseif !trainIsBrakingDownhill && resistingForceNegative
                            currentStepSize = settings.stepSize / 10.0^cycle

                        elseif trainIsBrakingDownhill && !resistingForceNegative
                            currentStepSize = settings.stepSize / 10.0^cycle

                        elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                            if settings.stepVariable == :distance
                                currentStepSize = nextPointOfInterest[:s] - drivingCourse[end-1][:s]
                            else
                                currentStepSize = settings.stepSize / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] > targetPosition # TODO also the following? drivingCourse[end][:s] > CS[:s_entry] + train.length))
                            if settings.stepVariable == :distance
                                currentStepSize = targetPosition - drivingCourse[end-1][:s]
                            else
                                currentStepSize = settings.stepSize / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] == targetPosition # || drivingCourse[end][:s]==CS[:s_exit]
                            break

                        elseif drivingCourse[end][:s] >= CS[:s_entry] + train.length
                            break

                        elseif drivingCourse[end][:s] == nextPointOfInterest[:s]
                            break

                        elseif !trainInPreviousCS
                            break

                        else
                            error("ERROR at cruising section: With the step variable ",settings.stepVariable," the while loop will be left although the if cases don't apply in CS",csId,"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                        end

                        # delete last support point for recalculating the last step with reduced step size
                        pop!(drivingCourse)

                        # conditions for the next for cycle
                        pointOfInterestReached = false
                        tractionDeficit = false
                        targetPositionReached = false
                        trainInPreviousCS = true
                        resistingForceNegative = drivingCourse[end][:F_R] < 0.0

                    else # if the level of approximation is reached
                        if drivingCourse[end][:s] > nextPointOfInterest[:s]
                            drivingCourse[end][:s] = nextPointOfInterest[:s] # round s down to nextPointOfInterest
                        elseif drivingCourse[end][:s] > targetPosition
                            if drivingMode != "clearing"
                                pop!(drivingCourse)
                            end
                        elseif drivingCourse[end][:s] == targetPosition
                            break
                        elseif drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                            break
                        elseif !trainIsBrakingDownhill && resistingForceNegative
                            break
                        elseif trainIsBrakingDownhill && !resistingForceNegative
                            break
                        elseif !trainInPreviousCS
                            break

                        else

                        end
                    end
                end #for

                if drivingCourse[end][:s] == nextPointOfInterest[:s]
                    drivingCourse[end][:label] = nextPointOfInterest[:label]
                end

            end #while
        end #if

        # conditions for the next while cycle
        targetPositionReached = drivingCourse[end][:s] >= targetPosition
        tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
        resistingForceNegative = drivingCourse[end][:F_R] < 0.0

        while !targetPositionReached && !tractionDeficit && (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            if nextPointOfInterest[:s] > targetPosition
                nextPointOfInterest = (s = targetPosition, label = "") #[targetPosition, ""]
            end

            # tractive effort (in N):
            if !trainIsBrakingDownhill
                drivingCourse[end][:F_T] = min(drivingCourse[end][:F_T], max(0.0, drivingCourse[end][:F_R]))
            else
                drivingCourse[end][:F_T] = 0.0
            end



            drivingCourse[end][:a] = 0.0    # acceleration (in m/s^2)

            # calculate the remaining cruising way
            #s_cruisingRemaining=targetPosition-drivingCourse[end][:s]
            s_cruisingRemaining = min(nextPointOfInterest[:s] -drivingCourse[end][:s], targetPosition -drivingCourse[end][:s])

            # create the next support point
            push!(drivingCourse, moveAStep(drivingCourse[end], :distance, s_cruisingRemaining, csId))
            drivingCourse[end][:behavior] = drivingMode
            if drivingCourse[end][:s] == nextPointOfInterest[:s]
                drivingCourse[end][:label] = nextPointOfInterest[:label]
            end

            calculateForces!(drivingCourse[end], CSs, csId, "default", train, settings.massModel)


            # conditions for the next while cycle
            targetPositionReached = drivingCourse[end][:s] >= targetPosition
            tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
            resistingForceNegative = drivingCourse[end][:F_R] < 0
        end #while

    end # else: return the characteristic section without a cruising section

    # set state flags
    stateFlags[:endOfCSReached] = drivingCourse[end][:s] == CS[:s_exit]
    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    stateFlags[:brakingStartReached] = brakingStartReached || drivingCourse[end][:s] + s_braking >= CS[:s_exit]
    stateFlags[:tractionDeficit] = tractionDeficit
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0.0
    lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
    stateFlags[:previousSpeedLimitReached] = lowestSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= lowestSpeedLimit[:v]
    stateFlags[:error] = !(targetPositionReached || tractionDeficit || !(cruisingType == "clearing" || ((cruisingType == "downhillBraking") == resistingForceNegative)))

    return (drivingCourse, stateFlags)
end #function addCruisingSection!


## This function calculates the support points for diminishing run when using maximum tractive effort and still getting slower
function addDiminishingSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    CS = CSs[csId]

    calculateForces!(drivingCourse[end], CSs, csId, "diminishing", train, settings.massModel)

    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

    # conditions for diminishing section
    targetSpeedReached = drivingCourse[end][:v] <= 0.0
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R] #|| stateFlags[:tractionDeficit]
    #s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]

    # use the conditions for the diminishing section
    if tractionDeficit && !targetSpeedReached && !endOfCSReached
        drivingMode = "diminishing"
        drivingCourse[end][:behavior] = drivingMode

        while tractionDeficit && !targetSpeedReached && !endOfCSReached && !brakingStartReached
            currentStepSize = settings.stepSize   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]

            for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while tractionDeficit && !brakingStartReached && !pointOfInterestReached && !targetSpeedReached
                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train.m_train_full, train.ξ_train)

                    # create the next support point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, csId))
                    drivingCourse[end][:behavior] = drivingMode

                    calculateForces!(drivingCourse[end], CSs, csId, drivingMode, train, settings.massModel)

                    # conditions for the next while cycle
                    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)

                    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
                    pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]
                    targetSpeedReached = drivingCourse[end][:v] <= 0.0
                    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                    endOfCSReached = drivingCourse[end][:s] == CS[:s_exit]
                end #while

                if csId==0
                    testFlag = true
                else
                    testFlag = false     # for testing
                end

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel+1
                    if drivingCourse[end][:v] < 0.0
                    #    if settings.stepVariable == :velocity
                    #        currentStepSize = drivingCourse[end-1][:v]
                    #    else
                            currentStepSize = settings.stepSize / 10.0^cycle
                    #    end
                    elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," > F_R=",drivingCourse[end][:F_R])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPOI=",nextPointOfInterest[:s])    # for testing
                        if settings.stepVariable == :distance
                            currentStepSize = nextPointOfInterest[:s] - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest[:s]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," == nextPOI=",nextPointOfInterest[:s])    # for testing
                        break

                    elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," == F_R=",drivingCourse[end][:F_R])    # for testing
                        break

                    elseif drivingCourse[end][:v] == 0.0
                        error("ERROR: The train stops during diminishing run in CS",csId," at position s=",drivingCourse[end][:s]," m because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    else
                        error("ERROR during diminishing run: With the step variable ",settings.stepVariable," the while loop will be left although s+s_braking<s_exit && v>0.0  in CS",csId,"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    brakingStartReached = false
                    pointOfInterestReached = false
                    targetSpeedReached = false
                    tractionDeficit = true
                    endOfCSReached = false

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: v=", drivingCourse[end][:v]," <= 0.0")    # for testing
                        error("ERROR: The train stops during diminishing run in CS",csId," because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        pop!(drivingCourse)

                        pointOfInterestReached = false
                        targetSpeedReached = false
                        tractionDeficit = true
                        endOfCSReached = false

                    elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest[:s]=",nextPointOfInterest[:s])    # for testing
                        drivingCourse[end][:s] = nextPointOfInterest[:s] # round s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," >= F_R=", drivingCourse[end][:F_R])    # for testing
                        break

                    else
                        testFlag && println("in CS",csId," diminishing cycle",cycle," case: else with v=", drivingCourse[end][:v]," > 0.0   and F_T=", drivingCourse[end][:F_T]," <= F_R=", drivingCourse[end][:F_R])    # for testing
                        #println("     and s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," <= s_exit=",CS[:s_exit])    # for testing
                        #println("     and s=", drivingCourse[end][:s]," <= nextPointOfInterest[:s]=",nextPointOfInterest[:s])    # for testing

                    #    if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                    #        brakingStartReached = true
                    #    end
                    end #if

                #    # TODO is it possible to put this into to the if-fork?
                #    if drivingCourse[end][:s] == CS[:s_exit]
                #        endOfCSReached = true
                #    end
                end #if
            end #for

            if drivingCourse[end][:s] == nextPointOfInterest[:s]
                drivingCourse[end][:label] = nextPointOfInterest[:label]
            end
        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = tractionDeficit
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_limit]
    stateFlags[:error] = !(endOfCSReached || brakingStartReached || !tractionDeficit)

    return (drivingCourse, stateFlags)
end #function addDiminishingSection!


## This function calculates the support points of the coasting section.
# Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the coasting section
function addCoastingSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    CS = CSs[csId]

    # conditions for coasting section
    lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
    previousSpeedLimitReached = lowestSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] > lowestSpeedLimit[:v]
    speedLimitReached = drivingCourse[end][:v] > CS[:v_limit]
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit] || previousSpeedLimitReached || speedLimitReached
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]

    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]

    # use the conditions for the coasting section
    if !targetSpeedReached && !endOfCSReached
       drivingMode = "coasting"
       drivingCourse[end][:behavior] = drivingMode

       while !targetSpeedReached && !endOfCSReached && !brakingStartReached
           currentStepSize = settings.stepSize  # initialize the step size that can be reduced near intersections
           nextPointOfInterest[:s] = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
           pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]

           for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !targetSpeedReached && !brakingStartReached && !pointOfInterestReached
                    if drivingCourse[end][:s] >= lowestSpeedLimit[:s_end]
                        # could be asked after creating an support point. This way here prevents even a minimal exceedance of speed limit.
                        lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
                    end

                   # traction effort and resisting forces (in N):
                   calculateForces!(drivingCourse[end], CSs, csId, drivingMode, train, settings.massModel)

                   # acceleration (in m/s^2):
                   drivingCourse[end][:a] = acceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train.m_train_full, train.ξ_train)

                   # create the next support point
                   push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, csId))
                   drivingCourse[end][:behavior] = drivingMode

                   # conditions for the next while cycle
                   s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                   brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
                   pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]
                   targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit] || drivingCourse[end][:v] > CS[:v_limit] ||  lowestSpeedLimit[:v] < CS[:v_limit] && (drivingCourse[end][:v] > lowestSpeedLimit[:v] || (drivingCourse[end][:v] == lowestSpeedLimit[:v] && drivingCourse[end][:s] < lowestSpeedLimit[:s_end]))
                end # while

                testFlag = false

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel+1
                   if drivingCourse[end][:s] + s_braking > CS[:s_exit]
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                       currentStepSize = settings.stepSize / 10.0^cycle

                   elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest[:s]=",nextPointOfInterest[:s])    # for testing
                       if settings.stepVariable == :distance
                           currentStepSize = nextPointOfInterest[:s] - drivingCourse[end-1][:s]
                       else
                           currentStepSize = settings.stepSize / 10.0^cycle
                       end

                   elseif drivingCourse[end][:v] < CS[:v_exit]  # TODO: if accelereation and coasting functions will be combined this case is only for coasting
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: v=", drivingCourse[end][:v]," < v_exit=", CS[:v_exit])    # for testing
                       if settings.stepVariable == :velocity
                           currentStepSize = drivingCourse[end-1][:v] - CS[:v_exit]
                       else
                           currentStepSize = settings.stepSize / 10.0^cycle
                       end

                   elseif drivingCourse[end][:v] > lowestSpeedLimit[:v]
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: v=", drivingCourse[end][:v]," > v_lowestLimit=",  lowestSpeedLimit[:v])    # for testing
                       if settings.stepVariable == :velocity
                           currentStepSize =  lowestSpeedLimit[:v] - drivingCourse[end-1][:v]
                       else
                           currentStepSize = settings.stepSize / 10.0^cycle
                       end

                   elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                       break

                   elseif drivingCourse[end][:v] == CS[:v_exit]
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: v=", drivingCourse[end][:v]," == v_exit=", CS[:v_exit])    # for testing
                       break

                   elseif drivingCourse[end][:s] == nextPointOfInterest[:s]
                       testFlag && println("in CS",csId," coasting cycle",cycle," case: s =", drivingCourse[end][:s]," > nextPointOfInterest[:s]=",nextPointOfInterest[:s])    # for testing
                       break

                   else
                       # TODO: not needed. just for testing
                       error("ERROR at coasting until braking section: With the step variable ",settings.stepVariable," the while loop will be left although v<v_limit and s+s_braking<s_exit in CS",csId,"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                   end
                   # delete last support point for recalculating the last step with reduced step size
                   pop!(drivingCourse)

                   # conditions for the next for cycle
                   brakingStartReached = false
                   pointOfInterestReached = false
                   targetSpeedReached = false


               else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        println("INFO: The train stops during the coasting section in CS",csId,"                                                                ",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N and s_braking=",s_braking,"m.")

                   elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                       # delete last support point because it went to far
                       pop!(drivingCourse)

                       # conditions for the next for cycle
                    #   brakingStartReached = true
                       pointOfInterestReached = false
                       targetSpeedReached = false

                   elseif drivingCourse[end][:v] > lowestSpeedLimit[:v] # if the train gets to fast it has to brake to hold the velocity limit
                       # delete last support point because it went to far
                       pop!(drivingCourse)

                       # conditions for the next for cycle
                       brakingStartReached = false
                       pointOfInterestReached = false
                       if lowestSpeedLimit[:v] != CS[:v_limit]
                           previousSpeedLimitReached = true
                       else
                           speedLimitReached = true
                       end

                   elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                       drivingCourse[end][:s] = nextPointOfInterest[:s] # round s down to nextPointOfInterest
                   else
                       # do nothing for example for drivingCourse[end][:s] + s_braking == CS[:s_exit]
                   end
               end
           end #for

           if drivingCourse[end][:s] == nextPointOfInterest[:s]
               drivingCourse[end][:label] = nextPointOfInterest[:label]
           end

       end #while
   end

   # set state flags
   stateFlags[:endOfCSReached] = endOfCSReached
   stateFlags[:brakingStartReached] = brakingStartReached
   stateFlags[:tractionDeficit] = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
   stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
   stateFlags[:previousSpeedLimitReached] = previousSpeedLimitReached
   stateFlags[:speedLimitReached] = speedLimitReached
   stateFlags[:error] = !(endOfCSReached || brakingStartReached || stateFlags[:tractionDeficit] || previousSpeedLimitReached || speedLimitReached)

   return (drivingCourse, stateFlags)
end #function addCoastingSection!


## This function calculates the support points of the braking section.
#    Therefore it gets its first support point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingSection!(drivingCourse::Vector{Dict}, stateFlags::Dict, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    CS = CSs[csId]

    # conditions for braking section
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]

    # use the conditions for the braking section
    if !targetSpeedReached && !endOfCSReached
        drivingMode = "braking"
        drivingCourse[end][:behavior] = drivingMode

        while !targetSpeedReached && !endOfCSReached
            currentStepSize = settings.stepSize  # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]

            for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !targetSpeedReached && !endOfCSReached && !pointOfInterestReached
                  # traction effort and resisting forces (in N):
                  calculateForces!(drivingCourse[end], CSs, csId, drivingMode, train, settings.massModel)

                  # acceleration (in m/s^2):
                  drivingCourse[end][:a] = train.a_braking
                  # TODO or: drivingCourse[end][:a] = brakingAcceleration(drivingCourse[end][:v], CS[:v_exit], CS[:s_exit]-drivingCourse[end][:s])

                  if settings.stepVariable == :distance && ((drivingCourse[end][:v]/drivingCourse[end][:a])^2+2*currentStepSize/drivingCourse[end][:a])<0.0 || (drivingCourse[end][:v]^2+2*currentStepSize*drivingCourse[end][:a])<0.0
                      # create empty support point and set it for the values of s_exit and v_exit
                      push!(drivingCourse, SupportPoint())
                      drivingCourse[end][:behavior] = drivingMode
                      recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                  else
                      # create the next support point
                      push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, csId))
                      drivingCourse[end][:behavior] = drivingMode
                  end

                  # conditions for the next while cycle
                  pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[:s]
                  endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit]
                  targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
                end # while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                # TODO: is there a better way than rounding like in the following?
                if cycle < settings.approxLevel+1
                    if drivingCourse[end][:v] < CS[:v_exit]
                        if settings.stepVariable == :velocity
                            currentStepSize = drivingCourse[end-1][:v] - CS[:v_exit]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                        if settings.stepVariable == :distance
                            currentStepSize = nextPointOfInterest[:s] - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] == CS[:v_exit] && drivingCourse[end][:s] == CS[:s_exit]
                        break

                    elseif drivingCourse[end][:v] == CS[:v_exit]
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        endOfCSReached = true
                        break

                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        targetSpeedReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest[:s]
                        break
                    end

                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    pointOfInterestReached = false
                    endOfCSReached = false
                    targetSpeedReached = false

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] < 0.0
                        # TODO: drivingCourse[end][:v] < CS[:v_exit] should be enough
                        # reset last point with setting v=v_exit. still possible with v_exit now meaning v_exitMax?
                    #   println("during braking section in CS",csId,": rounding v up from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                       recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], 0.0)
                       endOfCSReached = true
                       break
                   elseif drivingCourse[end][:s] > CS[:s_exit]
                #       println("during braking section in CS",csId,": rounding s down from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                      # recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                       drivingCourse[end][:s] = CS[:s_exit]
                       break
                    elseif drivingCourse[end][:s] > nextPointOfInterest[:s]
                        drivingCourse[end][:s] = nextPointOfInterest[:s] # round s down to nextPointOfInterest
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit] && drivingCourse[end][:s] == CS[:s_exit]
                        break
                    elseif drivingCourse[end][:v] < CS[:v_exit]
                        # reset last point with setting v=v_exit
                    #    println("during braking section in CS",csId,": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        endOfCSReached = true
                        break
                   elseif drivingCourse[end][:v] == CS[:v_exit]
                    #    println("during braking section in CS",csId,": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        endOfCSReached = true
                        break
                    elseif drivingCourse[end][:s] == CS[:s_exit]
                    #    println("during braking section in CS",csId,": rounding v down from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        targetSpeedReached = true
                        break
                    else
                        # do nothing for example for drivingCourse[end][:s]==nextPointOfInterest[:s]
                    end
                end
            end #for

            if drivingCourse[end][:s] == nextPointOfInterest[:s]
                drivingCourse[end][:label] = nextPointOfInterest[:label]
            end

        end #while
    end  # else: return the characteristic section without a braking section

    # set state flags
    lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
    stateFlags[:previousSpeedLimitReached] = lowestSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= lowestSpeedLimit[:v]
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_exit]
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:error] = !(endOfCSReached)
    calculateForces!(drivingCourse[end], CSs, csId, "default", train, settings.massModel)
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0

    return (drivingCourse, stateFlags)
end #function addBrakingSection!


## This function calculates the support point of the halt.
#    Therefore it gets its first support point and the characteristic section and returns the characteristic section including the halt if needed.
function addHalt!(drivingCourse::Vector{Dict}, CSs::Vector{Dict}, csId::Integer, settings::Settings, train::Train)
    # CS = CSs[csId]    # is not needed here
    if drivingCourse[end][:v] == 0.0
        drivingMode = "halt"
        drivingCourse[end][:behavior] = drivingMode

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, csId, drivingMode, train, settings.massModel)
    end  # else: return the characteristic section without a halt section section
    return drivingCourse
end #function addHalt!

function recalculateLastBrakingPoint!(drivingCourse, s_target, v_target)
    currentPoint = drivingCourse[end]
    previousPoint = drivingCourse[end-1]
    # set s and v
    currentPoint[:s] = s_target          # position (in m)
    currentPoint[:v] = v_target          # velocity (in m/s)

   # calculate other values
   previousPoint[:a] = brakingAcceleration(previousPoint[:v], currentPoint[:v], currentPoint[:s]-previousPoint[:s])
#    # TODO: just for testing
#    if previousPoint[:a]<train.a_braking || previousPoint[:a]>=0.0
#       println("Warning: a_braking gets to high in CS ",csId, "   with a=",previousPoint[:a]  ,"  >  ",train.a_braking)
#    end
   currentPoint[:t] = previousPoint[:t] + Δt_with_Δv(currentPoint[:v]-previousPoint[:v], previousPoint[:a])             # point in time (in s)
end #function recalculateLastBrakingPoint

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(CSs::Vector{Dict}, a_braking::Real, approxLevel::Integer)
    # limit the entry and exit velocities of the characteristic sections to secure that the train stops at the moving sections end

        csId = length(CSs)
        v_entryFollowing = 0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            # calculate the maximum possible entry velocity to define the previous section's maximum allowed exit velocity
            CS = CSs[csId]

            CS[:v_exit] = min(CS[:v_limit], v_entryFollowing)

            v_entry = brakingStartVelocity(CS[:v_exit], a_braking, CS[:s_exit]-CS[:s_entry], approxLevel)

            v_entryFollowing = min(CS[:v_limit], v_entry)

            csId = csId - 1
        end #while
    return CSs
end #function secureBrakingBehavior!
