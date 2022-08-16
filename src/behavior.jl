#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

## This function calculates the support points of the breakFree section.
# Therefore it gets its first support point and the characteristic section and returns the characteristic section including the behavior section for breakFree if needed.
# Info: currently the values of the breakFree section will be calculated like in the accelerating section
function addBreakFreeSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, settings::Settings, train::Train, CSs::Vector{Dict})
    # conditions for the break free section
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    trainIsHalting = drivingCourse[end][:v] == 0.0

    if trainIsHalting && !endOfCSReached
        drivingMode = "breakFree"
        drivingCourse[end][:behavior] = drivingMode
        startingPoint = drivingCourse[end][:i]

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, CS[:id], "accelerating", train, settings.massModel)    # currently the tractive effort is calculated like in the accelerating section

        # calculate the breakFree section with calculating the accelerating section and just using the first step and removing the rest
        try (CS, drivingCourse, stateFlags) = addAcceleratingSection!(CS, drivingCourse, stateFlags, settings, train, CSs)
        catch(acceleratingError)
            println("This error happened during the break free phase that is using the accelerating function:")
            rethrow(acceleratingError)
        end

        # delete every supportPoint except the first two
        while drivingCourse[end][:i] > startingPoint +1
            pop!(drivingCourse)
        end

        # change the accelerating data to break free
        drivingCourse[end-1][:behavior] = drivingMode
        drivingCourse[end][:behavior] = drivingMode
    end # else: return the characteristic section without a breakFree section

    # determine state flags

    if haskey(stateFlags, :usedForDefiningCharacteristics) && stateFlags[:usedForDefiningCharacteristics]
        s_braking = 0.0
    else
        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    end

    # reset state flags
    stateFlags[:endOfCSReached] = drivingCourse[end][:s] >= CS[:s_exit]
    stateFlags[:brakingStartReached] = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
    stateFlags[:tractionDeficit] = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]   # or add another flag for equal forces?
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = false
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_limit]
    stateFlags[:error] = !(stateFlags[:endOfCSReached] || stateFlags[:brakingStartReached] || stateFlags[:tractionDeficit] || stateFlags[:previousSpeedLimitReached] || stateFlags[:speedLimitReached])

    return (CS, drivingCourse, stateFlags)
end #function addBreakFreeSection!

## This function calculates the support points of the clearing section.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the clearing section.
function addClearingSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, settings::Settings, train::Train, CSs::Vector{Dict})
    if stateFlags[:previousSpeedLimitReached]
        currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train.length)

        if haskey(stateFlags, :usedForDefiningCharacteristics) && stateFlags[:usedForDefiningCharacteristics]
            ignoreBraking = true
            s_braking = 0.0
        else
            ignoreBraking = false
            s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
        end

        s_clearing = min(CS[:s_exit]-drivingCourse[end][:s]-s_braking, currentSpeedLimit[:s_end] - drivingCourse[end][:s])
        if s_clearing > 0.0
            (CS, drivingCourse, stateFlags) = addCruisingSection!(CS, drivingCourse, stateFlags, s_clearing, settings, train, CSs, "clearing")
            calculateForces!(drivingCourse[end], CSs, CS[:id], "accelerating", train, settings.massModel)
        #    stateFlags[:brakingStartReached] = brakingStartReached
        #    stateFlags[:endOfCSReached] = stateFlags[:endOfCSReached] || drivingCourse[end][:s] == CS[:s_exit]
        else
            error("ERROR: clearing <=0.0 although it has to be >0.0 in CS ",CS[:id])
        end
        #stateFlags[:previousSpeedLimitReached] = false
        currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train.length)
        stateFlags[:previousSpeedLimitReached] = currentSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= currentSpeedLimit[:v]
    else
        stateFlags[:error] = true
    end

    return (CS, drivingCourse, stateFlags)
end #function addClearingSection

## This function calculates the support points of the accelerating section.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the accelerating section
function addAcceleratingSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, settings::Settings, train::Train, CSs::Vector{Dict})
 #= TODO: instead of CS just give csId?
      -> CS = CSs[csId]      =#

    calculateForces!(drivingCourse[end], CSs, CS[:id], "accelerating", train, settings.massModel)

    if haskey(stateFlags, :usedForDefiningCharacteristics) && stateFlags[:usedForDefiningCharacteristics]
        ignoreBraking = true
        s_braking = 0.0
    else
        ignoreBraking = false
        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    end

    # conditions for the accelerating section
    targetSpeedReached = drivingCourse[end][:v] >= CS[:v_peak] || stateFlags[:speedLimitReached]
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]
    previousSpeedLimitReached = stateFlags[:previousSpeedLimitReached]

    # use the conditions for the accelerating section
    if !targetSpeedReached && !endOfCSReached && tractionSurplus && !brakingStartReached
        drivingMode = "accelerating"
        drivingCourse[end][:behavior] = drivingMode

        currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train.length)
        previousSpeedLimitReached = currentSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= currentSpeedLimit[:v]
        speedLimitReached = drivingCourse[end][:v] >= CS[:v_limit]
        #speedLimitReached = drivingCourse[end][:v] > currentSpeedLimit[:v]
            #targetSpeedReached = speedLimitReached
        while !targetSpeedReached && !endOfCSReached && tractionSurplus && !brakingStartReached && !previousSpeedLimitReached
            currentStepSize = settings.stepSize   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]

            for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                if !ignoreBraking
                    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                end

                while !targetSpeedReached && !speedLimitReached && !brakingStartReached && !pointOfInterestReached && tractionSurplus && !previousSpeedLimitReached
                # 03/08 old: while drivingCourse[end][:v] < CS[:v_peak] && drivingCourse[end][:v] <= currentSpeedLimit[:v] && !brakingStartReached && drivingCourse[end][:s] < nextPointOfInterest[1] && drivingCourse[end][:F_T] > drivingCourse[end][:F_R]      # as long as s_i + s_braking < s_CSexit
                    if drivingCourse[end][:s] >= currentSpeedLimit[:s_end]
                        # could be asked after creating an support point. This way here prevents even a minimal exceedance of speed limit will be noticed. On the other hand the train cruises possibly a little to long
                        currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train.length)
                    end

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train.m_train_full, train.ξ_train)

                    # create the next support point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = drivingMode

                    calculateForces!(drivingCourse[end], CSs, CS[:id], drivingMode, train, settings.massModel)

                    # conditions for the next while cycle
                    if !ignoreBraking
                        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                    end
                    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
                    speedLimitReached = drivingCourse[end][:v] > CS[:v_limit]
                    previousSpeedLimitReached  = currentSpeedLimit[:v] < CS[:v_limit] && (drivingCourse[end][:v] > currentSpeedLimit[:v] || (drivingCourse[end][:v] == currentSpeedLimit[:v] && drivingCourse[end][:s] < currentSpeedLimit[:s_end]))
                    targetSpeedReached = drivingCourse[end][:v] >= CS[:v_peak]
                        #targetSpeedReached = speedLimitReached
                    pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]   # POIs include s_exit as well
                    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                end #while

                if CS[:id]==0
                    testFlag = true
                else
                    testFlag = false     # for testing
                end

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel+1
                    if drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," <= F_R=",drivingCourse[end][:F_R])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPOI=",nextPointOfInterest[1])    # for testing
                        if settings.stepVariable == :distance
                            currentStepSize = nextPointOfInterest[1] - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > CS[:v_peak]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_peak=",CS[:v_peak])    # for testing
                        if settings.stepVariable == :velocity
                            currentStepSize = CS[:v_peak]-drivingCourse[end-1][:v]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > currentSpeedLimit[:v]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_limitCurrent=",currentSpeedLimit[:v])    # for testing
                        if settings.stepVariable == :velocity
                            currentStepSize = currentSpeedLimit[:v]-drivingCourse[end-1][:v]

                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                        if s_braking == 0.0
                            endOfCSReached = true
                        end
                        break

                    elseif drivingCourse[end][:v] == CS[:v_peak]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," == v_peak=",CS[:v_peak])    # for testing
                        break

                    elseif drivingCourse[end][:v] == currentSpeedLimit[:v] && drivingCourse[end][:s] < currentSpeedLimit[:s_end]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," == v_limitCurrent=",currentSpeedLimit[:v])    # for testing
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest[1]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," == nextPOI=",nextPointOfInterest[1])    # for testing
                        if nextPointOfInterest[1] == CS[:s_exit]
                            endOfCSReached = true
                        end
                        break

                    else
                        println("v=",drivingCourse[end][:v],"   v_peak= ", CS[:v_peak] , "  v_cLimit=", currentSpeedLimit[:v])
                        println("s=" ,drivingCourse[end][:s],"   s_exit=", CS[:s_exit], "   s+s_braking=", drivingCourse[end][:s] +s_braking,"   nextPOI=",nextPointOfInterest[1])
                        println("F_T=",drivingCourse[end][:F_T] ,"   F_R=", drivingCourse[end][:F_R])

                        error("ERROR at accelerating section: With the step variable ",settings.stepVariable," the while loop will be left although v<v_peak and s<s_exit in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                    end
                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    brakingStartReached = false
                    previousSpeedLimitReached = false
                    speedLimitReached = false
                    targetSpeedReached = false
                    endOfCSReached = false
                    pointOfInterestReached = false
                    tractionSurplus = true

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] > CS[:v_peak]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_peak=",CS[:v_peak])    # for testing
                        pop!(drivingCourse)

                        # conditions for the next section
                        brakingStartReached = false

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        if s_braking > 0.0
                            pop!(drivingCourse)

                        else
                            drivingCourse[end][:s] = CS[:s_exit] # round s down to CS[:s_exit]
                        end

                    elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest[1]=",nextPointOfInterest[1])    # for testing
                        drivingCourse[end][:s] = nextPointOfInterest[1] # round s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," <= F_R=",drivingCourse[end][:F_R])    # for testing

                    elseif drivingCourse[end][:v] > currentSpeedLimit[:v]
                        testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," > v_limitCurrent=",currentSpeedLimit[:v])    # for testing
                        previousSpeedLimitReached = true

                        pop!(drivingCourse)

                    else
                        if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                            testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," else case and there: s +s_braking=", drivingCourse[end][:s],",+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        elseif drivingCourse[end][:v] == currentSpeedLimit[:v]
                            testFlag && println("in CS",CS[:id]," accelerating cycle",cycle," case: v=", drivingCourse[end][:v]," == v_limitCurrent=",currentSpeedLimit[:v])    # for testing

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
            if drivingCourse[end][:s] == nextPointOfInterest[1]
                drivingCourse[end][:label] = nextPointOfInterest[2]
            end

        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = !(tractionSurplus || drivingCourse[end][:F_T] == drivingCourse[end][:F_R])   # or add another flag for equal forces?
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = previousSpeedLimitReached
    stateFlags[:speedLimitReached] = targetSpeedReached
    stateFlags[:error] = !(endOfCSReached || brakingStartReached || stateFlags[:tractionDeficit] || previousSpeedLimitReached || targetSpeedReached)

    return (CS, drivingCourse, stateFlags)
end #function addAcceleratingSection!


## This function calculates the support points of the cruising section.
#   Therefore it gets its first support point and the characteristic section and returns the characteristic section including the behavior section for cruising if needed.
function addCruisingSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, s_cruising::Real, settings::Settings, train::Train, CSs::Vector{Dict}, cruisingType::String)
    trainIsClearing = cruisingType == "clearing"
    trainIsBrakingDownhill = cruisingType == "downhillBraking"

    # traction effort and resisting forces (in N)
    if !trainIsBrakingDownhill  # TODO: or just give drivingMode instead of "cruising"/"braking"?
        calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings.massModel)
    else
        calculateForces!(drivingCourse[end], CSs, CS[:id], "braking", train, settings.massModel)
    end

    if haskey(stateFlags, :usedForDefiningCharacteristics) && stateFlags[:usedForDefiningCharacteristics]
        ignoreBraking = true
        s_braking = 0.0
    else
        ignoreBraking = false
        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    end

    # conditions for cruising section
    #s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]
    speedIsValid = drivingCourse[end][:v]>0.0 && drivingCourse[end][:v]<=CS[:v_peak]
    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
    targetPositionReached = s_cruising == 0.0
    resistingForceNegative = drivingCourse[end][:F_R] < 0

    if speedIsValid && !brakingStartReached && !tractionDeficit && !targetPositionReached
     # 03/04 old: if drivingCourse[end][:v]>0.0 && drivingCourse[end][:v]<=CS[:v_peak] && !brakingStartReached && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
        drivingMode = cruisingType
        drivingCourse[end][:behavior] = drivingMode
        # TODO: necessary?
        targetPosition = min(drivingCourse[end][:s] + s_cruising, CS[:s_exit])
        # 07/12 old: s_cruising = min(s_cruising, CS[:s_exit]-drivingCourse[end][:s])

        # traction effort and resisting forces (in N)
        if !trainIsBrakingDownhill
            calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings.massModel)
        else
            calculateForces!(drivingCourse[end], CSs, CS[:id], "braking", train, settings.massModel)
        end

        if settings.massModel == :homogeneous_strip && CS[:id] > 1
            # conditions for cruising section
            trainInPreviousCS = drivingCourse[end][:s] < CS[:s_entry] + train.length
            targetPositionReached = drivingCourse[end][:s] >= targetPosition
            resistingForceNegative = drivingCourse[end][:F_R] < 0.0
#            targetSpeedReached = stateFlags[:speedLimitReached] || drivingCourse[end][:v] >= CS[:v_peak]
                # TODO: change? to correctCruisingType = (trainIsClearing || (trainIsBrakingDownhill == drivingCourse[end][:F_R] < 0)) # while clearing tractive or braking force can be used
#&& targetSpeedReached
            # use the conditions for the cruising section
            while trainInPreviousCS && !targetPositionReached && !tractionDeficit && (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
                currentStepSize = settings.stepSize
                nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
                pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]

                for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                    while trainInPreviousCS && !targetPositionReached && !pointOfInterestReached && !tractionDeficit && (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
                     # the tractive effort is lower than the resisiting forces and the train has use the highest possible effort to try to stay at v_peak OR the mass model homogeneous strip is used and parts of the train are still in former CS
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
                            push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, CS[:id]))
                        else
                            push!(drivingCourse, moveAStep(drivingCourse[end], position, train.length/(10.0^cycle), CS[:id])) # TODO which step size should be used?
                        end
                        drivingCourse[end][:behavior] = drivingMode

                        # traction effort and resisting forces (in N)
                        calculateForces!(drivingCourse[end], CSs, CS[:id], "default", train, settings.massModel)
#                        calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings.massModel)
                        #if !trainIsBrakingDownhill
                        #    calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings.massModel)
                        #else
                        #    calculateForces!(drivingCourse[end], CSs, CS[:id], "braking", train, settings.massModel)
                        #end

                        # conditions for the next while cycle
                        pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]   # POIs include s_exit as well
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

                        elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                            if settings.stepVariable == :distance
                                currentStepSize = nextPointOfInterest[1] - drivingCourse[end-1][:s]
                            else
                                currentStepSize = settings.stepSize / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] > targetPosition # TODO also the following? drivingCourse[end][:s] > CSs[CS[:id]][:s_entry] + train.length))
                            if settings.stepVariable == :distance
                                currentStepSize = targetPosition - drivingCourse[end-1][:s]
                            else
                                currentStepSize = settings.stepSize / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] == targetPosition # || drivingCourse[end][:s]==CS[:s_exit]
                            break

                        elseif drivingCourse[end][:s] >= CS[:s_entry] + train.length
                            break

                        elseif drivingCourse[end][:s] == nextPointOfInterest[1]
                            break

                        elseif !trainInPreviousCS
                            break

                        else
                            error("ERROR at cruising section: With the step variable ",settings.stepVariable," the while loop will be left although the if cases don't apply in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
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
                        if drivingCourse[end][:s] > nextPointOfInterest[1]
                            drivingCourse[end][:s] = nextPointOfInterest[1] # round s down to nextPointOfInterest
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

                if drivingCourse[end][:s] == nextPointOfInterest[1]
                    drivingCourse[end][:label] = nextPointOfInterest[2]
                end

            end #while
        end #if

        # conditions for the next while cycle
        targetPositionReached = drivingCourse[end][:s] >= targetPosition
        tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
        resistingForceNegative = drivingCourse[end][:F_R] < 0.0

        while !targetPositionReached && !tractionDeficit && (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
        # 03/09 old: while drivingCourse[end][:s] < targetPosition && drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            if nextPointOfInterest[1] > targetPosition
                nextPointOfInterest = [targetPosition, ""]
            end

            # tractive effort (in N):
#03/25            drivingCourse[end][:F_T] = min(drivingCourse[end][:F_T], max(0.0, drivingCourse[end][:F_R]))
            if !trainIsBrakingDownhill
                drivingCourse[end][:F_T] = min(drivingCourse[end][:F_T], max(0.0, drivingCourse[end][:F_R]))
            else
                drivingCourse[end][:F_T] = 0.0
            end



            drivingCourse[end][:a] = 0.0    # acceleration (in m/s^2)

            # calculate the remaining cruising way
            #s_cruisingRemaining=targetPosition-drivingCourse[end][:s]
            s_cruisingRemaining = min(nextPointOfInterest[1] -drivingCourse[end][:s], targetPosition -drivingCourse[end][:s])

            # create the next support point
            push!(drivingCourse, moveAStep(drivingCourse[end], :distance, s_cruisingRemaining, CS[:id]))
            drivingCourse[end][:behavior] = drivingMode
            if drivingCourse[end][:s] == nextPointOfInterest[1]
                drivingCourse[end][:label] = nextPointOfInterest[2]
            end

            calculateForces!(drivingCourse[end], CSs, CS[:id], "default", train, settings.massModel)
#            calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings.massModel)
            #if !trainIsBrakingDownhill
            #    calculateForces!(drivingCourse[end], CSs, CS[:id], "cruising", train, settings.massModel)
            #else
            #    calculateForces!(drivingCourse[end], CSs, CS[:id], "braking", train, settings.massModel)
            #end

            # conditions for the next while cycle
            targetPositionReached = drivingCourse[end][:s] >= targetPosition
            tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
            resistingForceNegative = drivingCourse[end][:F_R] < 0
        end #while

    end # else: return the characteristic section without a cruising section

    # set state flags
    stateFlags[:endOfCSReached] = drivingCourse[end][:s] == CS[:s_exit]
    if !ignoreBraking
        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    end
    stateFlags[:brakingStartReached] = brakingStartReached || drivingCourse[end][:s] + s_braking >= CS[:s_exit]
    stateFlags[:tractionDeficit] = tractionDeficit
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0.0
    currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train.length)
    stateFlags[:previousSpeedLimitReached] = currentSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= currentSpeedLimit[:v]
    stateFlags[:error] = !(targetPositionReached || tractionDeficit || !(cruisingType == "clearing" || ((cruisingType == "downhillBraking") == resistingForceNegative)))

    return (CS, drivingCourse, stateFlags)
end #function addCruisingSection!


## This function calculates the support points for diminishing run when using maximum tractive effort and still getting slower
function addDiminishingSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, settings::Settings, train::Train, CSs::Vector{Dict})
    calculateForces!(drivingCourse[end], CSs, CS[:id], "diminishing", train, settings.massModel)

    if haskey(stateFlags, :usedForDefiningCharacteristics) && stateFlags[:usedForDefiningCharacteristics]
        ignoreBraking = true
        s_braking = 0.0
    else
        ignoreBraking = false
        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    end

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
            currentStepSize=settings.stepSize   # initialize the step size that can be reduced near intersections
            nextPointOfInterest = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]

            for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while tractionDeficit && !brakingStartReached && !pointOfInterestReached && !targetSpeedReached
                # 03/09 old: while drivingCourse[end][:F_T] < drivingCourse[end][:F_R] && !brakingStartReached && drivingCourse[end][:s] < nextPointOfInterest[1] && drivingCourse[end][:v]>0.0       # as long as s_i + s_braking < s_end
                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train.m_train_full, train.ξ_train)

                    # create the next support point
                    push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, CS[:id]))
                    drivingCourse[end][:behavior] = drivingMode

                    calculateForces!(drivingCourse[end], CSs, CS[:id], drivingMode, train, settings.massModel)

                    # conditions for the next while cycle
                    if !ignoreBraking
                        s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                    end
                    brakingStartReached = drivingCourse[end][:s] +s_braking >= CS[:s_exit]
                    pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]
                    targetSpeedReached = drivingCourse[end][:v] <= 0.0
                    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                    endOfCSReached = drivingCourse[end][:s] == CS[:s_exit]
                end #while

                if CS[:id]==0
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
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," > F_R=",drivingCourse[end][:F_R])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPOI=",nextPointOfInterest[1])    # for testing
                        if settings.stepVariable == :distance
                            currentStepSize = nextPointOfInterest[1] - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                        break

                    elseif drivingCourse[end][:s] == nextPointOfInterest[1]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," == nextPOI=",nextPointOfInterest[1])    # for testing
                        break

                    elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," == F_R=",drivingCourse[end][:F_R])    # for testing
                        break

                    elseif drivingCourse[end][:v] == 0.0
                        error("ERROR: The train stops during diminishing run in CS",CS[:id]," at position s=",drivingCourse[end][:s]," m because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    else
                        error("ERROR during diminishing run: With the step variable ",settings.stepVariable," the while loop will be left although s+s_braking<s_exit && v>0.0  in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
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
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: v=", drivingCourse[end][:v]," <= 0.0")    # for testing
                        error("ERROR: The train stops during diminishing run in CS",CS[:id]," because the maximum tractive effort is lower than the resistant forces.",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N.")

                    elseif s_braking > 0.0 && drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                        pop!(drivingCourse)

                        pointOfInterestReached = false
                        targetSpeedReached = false
                        tractionDeficit = true
                        endOfCSReached = false

                    elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest[1]=",nextPointOfInterest[1])    # for testing
                        drivingCourse[end][:s] = nextPointOfInterest[1] # round s down to nextPointOfInterest

                    elseif drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: F_T=", drivingCourse[end][:F_T]," >= F_R=", drivingCourse[end][:F_R])    # for testing
                        break

                    else
                        testFlag && println("in CS",CS[:id]," diminishing cycle",cycle," case: else with v=", drivingCourse[end][:v]," > 0.0   and F_T=", drivingCourse[end][:F_T]," <= F_R=", drivingCourse[end][:F_R])    # for testing
                        #println("     and s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," <= s_exit=",CS[:s_exit])    # for testing
                        #println("     and s=", drivingCourse[end][:s]," <= nextPointOfInterest[1]=",nextPointOfInterest[1])    # for testing

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

            if drivingCourse[end][:s] == nextPointOfInterest[1]
                drivingCourse[end][:label] = nextPointOfInterest[2]
            end
        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = tractionDeficit
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_peak]
    stateFlags[:error] = !(endOfCSReached || brakingStartReached || !tractionDeficit)

    return (CS, drivingCourse, stateFlags)
end #function addDiminishingSection!


## This function calculates the support points of the coasting section.
# Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the coasting section
function addCoastingSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, settings::Settings, train::Train, CSs::Vector{Dict})
    # TODO: if the rear of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
            # with getCurrentSpeedLimit

    # conditions for coasting section
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]

    s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] || stateFlags[:brakingStartReached]

    # use the conditions for the coasting section
    if !targetSpeedReached && !endOfCSReached
       drivingMode = "coasting"
       drivingCourse[end][:behavior] = drivingMode

       while !targetSpeedReached && !endOfCSReached && !brakingStartReached
           currentStepSize=settings.stepSize  # initialize the step size that can be reduced near intersections
           nextPointOfInterest[1] = getNextPointOfInterest(CS[:pointsOfInterest], drivingCourse[end][:s])
           pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]

           for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !targetSpeedReached && !brakingStartReached && !pointOfInterestReached
                # 03/09 old : while drivingCourse[end][:v] > CS[:v_exit] && drivingCourse[end][:v] <= CS[:v_peak] && !brakingStartReached && drivingCourse[end][:s] < nextPointOfInterest[1]
                   # traction effort and resisting forces (in N):
                   calculateForces!(drivingCourse[end], CSs, CS[:id], drivingMode, train, settings.massModel)

                   # acceleration (in m/s^2):
                   drivingCourse[end][:a] = acceleration(drivingCourse[end][:F_T], drivingCourse[end][:F_R], train.m_train_full, train.ξ_train)

                   # create the next support point
                   push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, CS[:id]))
                   drivingCourse[end][:behavior] = drivingMode

                   # conditions for the next while cycle
                   s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
                   brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
                   pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]
                   targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit] || drivingCourse[end][:v] > CS[:v_peak]
                end # while

                testFlag = false

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel+1
                   if drivingCourse[end][:s] + s_braking > CS[:s_exit]
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," > s_exit=",CS[:s_exit])    # for testing
                       currentStepSize = settings.stepSize / 10.0^cycle

                   elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: s=", drivingCourse[end][:s]," > nextPointOfInterest[1]=",nextPointOfInterest[1])    # for testing

                       if settings.stepVariable == :distance
                           currentStepSize = nextPointOfInterest[1] - drivingCourse[end-1][:s]
                       else
                           currentStepSize = settings.stepSize / 10.0^cycle
                       end

                   elseif drivingCourse[end][:v] < CS[:v_exit]  # TODO: if accelereation and coasting functions will be combined this case is only for coasting
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: v=", drivingCourse[end][:v]," < v_exit=", CS[:v_exit])    # for testing
                        if settings.stepVariable == :velocity
                            currentStepSize = drivingCourse[end-1][:v] - CS[:v_exit]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end
                   elseif drivingCourse[end][:v] > CS[:v_peak]
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: v=", drivingCourse[end][:v]," > v_peak=", CS[:v_peak])    # for testing
                       if settings.stepVariable == :velocity
                           currentStepSize = CS[:v_peak] - drivingCourse[end-1][:v]
                       else
                           currentStepSize = settings.stepSize / 10.0^cycle
                       end
                   elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: s +s_braking=", drivingCourse[end][:s],"+",s_braking," = ",drivingCourse[end][:s] +s_braking," == s_exit=",CS[:s_exit])    # for testing
                       break

                   elseif drivingCourse[end][:v] == CS[:v_exit]
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: v=", drivingCourse[end][:v]," == v_exit=", CS[:v_exit])    # for testing
                       break

                   elseif drivingCourse[end][:s] == nextPointOfInterest[1]
                       testFlag && println("in CS",CS[:id]," coasting cycle",cycle," case: s =", drivingCourse[end][:s]," > nextPointOfInterest[1]=",nextPointOfInterest[1])    # for testing
                       break

                   else
                       # TODO: not needed. just for testing
                       error("ERROR at coasting until braking section: With the step variable ",settings.stepVariable," the while loop will be left although v<v_peak and s+s_braking<s_exit in CS",CS[:id],"  with s=" ,drivingCourse[end][:s]," m and v=",drivingCourse[end][:v]," m/s")
                   end
                   # delete last support point for recalculating the last step with reduced step size
                   pop!(drivingCourse)

                   # conditions for the next for cycle
                   brakingStartReached = false
                   pointOfInterestReached = false
                   targetSpeedReached = false


               else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        println("INFO: The train stops during the coasting section in CS",CS[:id],"                                                                ",
                        "       Before the stop the last point has the values s=",drivingCourse[end-1][:s]," m  v=",drivingCourse[end-1][:v]," m/s  a=",drivingCourse[end-1][:a]," m/s^2",
                        "       F_T=",drivingCourse[end-1][:F_T]," N  R_traction=",drivingCourse[end-1][:R_traction]," N  R_wagons=",drivingCourse[end-1][:R_wagons]," N  R_path=",drivingCourse[end-1][:R_path]," N and s_braking=",s_braking,"m.")

                   elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                       # delete last support point because it went to far
                       pop!(drivingCourse)

                       # conditions for the next for cycle
                    #   brakingStartReached = true
                       pointOfInterestReached = false
                       targetSpeedReached = false

                   elseif drivingCourse[end][:v] > CS[:v_peak] # if the train gets to fast it has to brake  # TODO: if accelereation and coasting functions will be combined this case is different for coasting and also the order of if cases is different
                       # delete last support point because it went to far
                       pop!(drivingCourse)

                       # conditions for the next for cycle
                       brakingStartReached = false
                       pointOfInterestReached = false
                     #  targetSpeedReached = true

                   elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                       drivingCourse[end][:s] = nextPointOfInterest[1] # round s down to nextPointOfInterest
                   else
                       # do nothing for example for drivingCourse[end][:s] + s_braking == CS[:s_exit]
                   end
               end
           end #for

           if drivingCourse[end][:s] == nextPointOfInterest[1]
               drivingCourse[end][:label] = nextPointOfInterest[2]
           end

       end #while

       stateFlags[:speedLimitReached] = false
   end

   # set state flags
   stateFlags[:endOfCSReached] = endOfCSReached
   stateFlags[:brakingStartReached] = brakingStartReached
   stateFlags[:tractionDeficit] = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
   stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
   stateFlags[:error] = !(endOfCSReached || brakingStartReached || stateFlags[:tractionDeficit] || previousSpeedLimitReached || targetSpeedReached)

   return (CS, drivingCourse, stateFlags)
end #function addCoastingSection!


## This function calculates the support points of the braking section.
#    Therefore it gets its first support point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingSection!(CS::Dict, drivingCourse::Vector{Dict}, stateFlags::Dict, settings::Settings, train::Train, CSs::Vector{Dict})
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
            pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]

            for cycle in 1:settings.approxLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !targetSpeedReached && !endOfCSReached && !pointOfInterestReached
                # 03/09 old: while drivingCourse[end][:v] > CS[:v_exit] && !targetSpeedReached && drivingCourse[end][:s] < CS[:s_exit] && drivingCourse[end][:s] < nextPointOfInterest[1]
                  # traction effort and resisting forces (in N):
                  calculateForces!(drivingCourse[end], CSs, CS[:id], drivingMode, train, settings.massModel)

                  # acceleration (in m/s^2):
                  drivingCourse[end][:a] = train.a_braking
                  # TODO or: drivingCourse[end][:a] = brakingAcceleration(drivingCourse[end][:v], CS[:v_exit], CS[:s_exit]-drivingCourse[end][:s])

                  if settings.stepVariable == :distance && ((drivingCourse[end][:v]/drivingCourse[end][:a])^2+2*currentStepSize/drivingCourse[end][:a])<0.0 || (drivingCourse[end][:v]^2+2*currentStepSize*drivingCourse[end][:a])<0.0
                      # create empty support point and set it for the values of s_exit and v_exit
                      push!(drivingCourse, SupportPoint())
                      drivingCourse[end][:i] = drivingCourse[end-1][:i]+1
                      drivingCourse[end][:behavior] = drivingMode
                      recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                  else
                      # create the next support point
                      push!(drivingCourse, moveAStep(drivingCourse[end], settings.stepVariable, currentStepSize, CS[:id]))
                      drivingCourse[end][:behavior] = drivingMode
                  end
                  #println(drivingCourse[end][:i],".  s=",drivingCourse[end][:s],"  s_exit=", CS[:s_exit],"  v_exit=", CS[:v_exit],"  v=",drivingCourse[end][:v])

                  # conditions for the next while cycle
                  pointOfInterestReached = drivingCourse[end][:s] >= nextPointOfInterest[1]
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
                    elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                        if settings.stepVariable == :distance
                            currentStepSize = nextPointOfInterest[1] - drivingCourse[end-1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end
                    elseif drivingCourse[end][:v] == CS[:v_exit] && drivingCourse[end][:s] == CS[:s_exit]
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit]
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        endOfCSReached = true
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                    elseif drivingCourse[end][:s] == CS[:s_exit]
                    #    println("during braking section in CS",CS[:id],": rounding v down from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        targetSpeedReached = true
                    #    println("  with a=", drivingCourse[end-1][:a]) # for testing
                        break
                    elseif drivingCourse[end][:s] == nextPointOfInterest[1]
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
                        # reset last point with setting v=v_exit
                    #   println("during braking section in CS",CS[:id],": rounding v up from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                       recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], 0.0)
                       endOfCSReached = true
                       break
                   elseif drivingCourse[end][:s] > CS[:s_exit]
                #       println("during braking section in CS",CS[:id],": rounding s down from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                      # recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                       drivingCourse[end][:s] = CS[:s_exit]
                       break
                    elseif drivingCourse[end][:s] > nextPointOfInterest[1]
                        drivingCourse[end][:s] = nextPointOfInterest[1] # round s down to nextPointOfInterest
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit] && drivingCourse[end][:s] == CS[:s_exit]
                        break
                    elseif drivingCourse[end][:v] < CS[:v_exit]
                        # reset last point with setting v=v_exit
                    #    println("during braking section in CS",CS[:id],": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        endOfCSReached = true
                        break
                   elseif drivingCourse[end][:v] == CS[:v_exit]
                    #    println("during braking section in CS",CS[:id],": rounding s up from ", drivingCourse[end][:s] ," to ", CS[:s_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        endOfCSReached = true
                        break
                    elseif drivingCourse[end][:s] == CS[:s_exit]
                    #    println("during braking section in CS",CS[:id],": rounding v down from ", drivingCourse[end][:v] ," to ", CS[:v_exit]) # for testing
                        recalculateLastBrakingPoint!(drivingCourse, CS[:s_exit], CS[:v_exit])
                        targetSpeedReached = true
                        break
                    else
                        # do nothing for example for drivingCourse[end][:s]==nextPointOfInterest[1]
                    end
                end
            end #for

            if drivingCourse[end][:s] == nextPointOfInterest[1]
                drivingCourse[end][:label] = nextPointOfInterest[2]
            end

        end #while
    end  # else: return the characteristic section without a braking section

    # set state flags
    currentSpeedLimit = getCurrentSpeedLimit(CSs, CS[:id], drivingCourse[end][:s], train.length)
    stateFlags[:previousSpeedLimitReached] = currentSpeedLimit[:v] != CS[:v_limit] && drivingCourse[end][:v] >= currentSpeedLimit[:v]
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_exit]
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:error] = !(endOfCSReached)
    calculateForces!(drivingCourse[end], CSs, CS[:id], "default", train, settings.massModel)
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0

    return (CS, drivingCourse, stateFlags)
end #function addBrakingSection!


## This function calculates the support point of the halt.
#    Therefore it gets its first support point and the characteristic section and returns the characteristic section including the halt if needed.
function addHalt!(CS::Dict, drivingCourse::Vector{Dict}, settings::Settings, train::Train, CSs::Vector{Dict})
    if drivingCourse[end][:v] == 0.0
        drivingMode = "halt"
        drivingCourse[end][:behavior] = drivingMode

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], CSs, CS[:id], drivingMode, train, settings.massModel)
    end  # else: return the characteristic section without a halt section section
    return (CS, drivingCourse)
end #function addHalt!

function recalculateLastBrakingPoint!(drivingCourse, s_target, v_target)
    currentPoint = drivingCourse[end]
    previousPoint = drivingCourse[end-1]
   # set s and v
   currentPoint[:s] = s_target                                          # position (in m)
   currentPoint[:v] = v_target                                          # velocity (in m/s)

   # calculate other values
   previousPoint[:a] = brakingAcceleration(previousPoint[:v], currentPoint[:v], currentPoint[:s]-previousPoint[:s])
#    # TODO: just for testing
#    if previousPoint[:a]<train.a_braking || previousPoint[:a]>=0.0
#       println("Warning: a_braking gets to high in CS ",CS[:id], "   with a=",previousPoint[:a]  ,"  >  ",train.a_braking)
#    end
   currentPoint[:t] = previousPoint[:t] + Δt_with_Δv(currentPoint[:v]-previousPoint[:v], previousPoint[:a])             # point in time (in s)
end #function recalculateLastBrakingPoint

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(CSs::Vector{Dict}, a_braking::Real, approxLevel::Integer)
    # limit the entry and exit velocity of the characteristic sections to secure that the train stops at the moving sections end

        csId = length(CSs)
        followingCSv_entry = 0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            CS = CSs[csId]

            CS[:v_exit] = min(CS[:v_limit], followingCSv_entry)

            v_entryMax = brakingStartVelocity(CS[:v_exit], a_braking, CS[:s_exit]-CS[:s_entry], approxLevel)

            CS[:v_entry] = min(CS[:v_limit], v_entryMax)
            CS[:v_peak] = CS[:v_entry]

            followingCSv_entry = CS[:v_entry]
            csId = csId - 1
        end #while
    return CSs
end #function secureBrakingBehavior!

## define the intersection velocities between the characterisitc sections to secure accelerating behavior
function secureAcceleratingBehavior!(CSs::Vector{Dict}, settings::Settings, train::Train)
    # limit the entry and exit velocity of the characteristic sections in case that the train accelerates in every section and cruises afterwards

    CSs[1][:v_entry] = 0.0     # the entry velocity of the first characteristic section is 0.0 m/s
    startingPoint = SupportPoint()
    startingPoint[:i] = 1

    previousCSv_exit = CSs[1][:v_entry]
    for CS in CSs
        CS[:v_entry] = min(CS[:v_entry], previousCSv_exit)
        startingPoint[:s] = CS[:s_entry]
        startingPoint[:v] = CS[:v_entry]
        calculateForces!(startingPoint, CSs, CS[:id], "accelerating", train, settings.massModel) # traction effort and resisting forces (in N)
        acceleratingCourse::Vector{Dict} = [startingPoint]    # List of support points

        if CS[:v_entry] < CS[:v_peak]
            # conditions for entering the accelerating phase
            stateFlags = Dict(:endOfCSReached => false,
                              :brakingStartReached => false,
                              :tractionDeficit => false,
                              :resistingForceNegative => false,
                              :previousSpeedLimitReached => false,
                              :speedLimitReached => false,
                              :error => false,
                              :usedForDefiningCharacteristics => true)      # because usedForDefiningCharacteristics == true the braking distance will be ignored during securing the accelerating phase
            v_peak = CS[:v_entry]
            (CS, acceleratingCourse, stateFlags) = addBreakFreeSection!(CS, acceleratingCourse, stateFlags, settings, train, CSs)
            while !stateFlags[:speedLimitReached] && !stateFlags[:endOfCSReached]
                if !stateFlags[:tractionDeficit]
                    if !stateFlags[:previousSpeedLimitReached]
                    (CS, acceleratingCourse, stateFlags) = addAcceleratingSection!(CS, acceleratingCourse, stateFlags, settings, train, CSs)        # this function changes the acceleratingCourse

                    elseif stateFlags[:previousSpeedLimitReached]
                        (CS, acceleratingCourse, stateFlags) = addClearingSection!(CS, acceleratingCourse, stateFlags, settings, train, CSs)        # this function is needed in case the train is not allowed to accelerate because of a previous speed limit
                    end
                else
                    if settings.massModel == :mass_point || acceleratingCourse[end][:s] > CS[:s_entry] + train.length
                        break
                    else
                        (CS, acceleratingCourse, stateFlags) = addDiminishingSection!(CS, acceleratingCourse, stateFlags, settings, train, CSs)        # this function is needed in case the resisitng forces are higher than the maximum possible tractive effort
                    end
                end
                v_peak = max(v_peak, acceleratingCourse[end][:v])
            end

            # CS[:v_peak] = max(CS[:v_entry], acceleratingCourse[end][:v])
            CS[:v_peak] = v_peak
            CS[:v_exit] = min(CS[:v_exit], CS[:v_peak], acceleratingCourse[end][:v])
        else #CS[:v_entry] == CS[:v_peak]
            # v_exit stays the same
        end #if

        previousCSv_exit = CS[:v_exit]
    end #for

    return CSs
end #function secureAcceleratingBehavior!
