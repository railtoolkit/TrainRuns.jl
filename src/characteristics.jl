#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

## create a moving section and its containing characteristic sections with secured braking, accelerating and cruising behavior
function determineCharacteristics(path::Path, train::Train, settings::Settings)
    movingSection = createMovingSection(path, train.v_limit, train.length)
    movingSection = secureBrakingBehavior!(movingSection, train.a_braking, settings.approxLevel)
    movingSection = secureAcceleratingBehavior!(movingSection, settings, train)
    #movingSection = secureCruisingBehavior!(movingSection, settings, train)

    return movingSection
end #function determineCharacteristics

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(movingSection::Dict, a_braking::Real, approxLevel::Integer)
    # this function limits the entry and exit velocity of the characteristic sections to secure that the train stops at the moving sections end
        CSs = movingSection[:characteristicSections]

        csId = length(CSs)
        followingCSv_entry = 0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            CS = CSs[csId]

            CS[:v_exit] = min(CS[:v_limit], followingCSv_entry)

            v_entryMax = calcBrakingStartVelocity(CS[:v_exit], a_braking, CS[:length], approxLevel)

            CS[:v_entry] = min(CS[:v_limit], v_entryMax)
            CS[:v_peak] = CS[:v_entry]


            # reset the characteristic section (CS), delete behavior sections (BS) that were used during the preperation for setting v_entry, v_peak and v_exit
            CS[:behaviorSections] = Dict()
            CS[:t] = 0.0

            followingCSv_entry = CS[:v_entry]
            csId = csId - 1
        end #while
    return movingSection
end #function secureBrakingBehavior!

## define the intersection velocities between the characterisitc sections to secure accelerating behavior
function secureAcceleratingBehavior!(movingSection::Dict, settings::Settings, train::Train)
    # this function limits the entry and exit velocity of the characteristic sections in case that the train accelerates in every section and cruises aterwards
    CSs = movingSection[:characteristicSections]

    CSs[1][:v_entry] = 0.0     # the entry velocity of the first characteristic section is 0.0 m/s
    startingPoint = createDataPoint()
    startingPoint[:i] = 1

    previousCSv_exit = CSs[1][:v_entry]
    for CS in CSs
        CS[:v_entry] = min(CS[:v_entry], previousCSv_exit)
        startingPoint[:s] = CS[:s_entry]
        startingPoint[:v] = CS[:v_entry]
        calculateForces!(startingPoint, CSs, CS[:id], "accelerating", train, settings.massModel) # traction effort and resisting forces (in N)
        acceleratingCourse::Vector{Dict} = [startingPoint]    # List of data points

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

#            CS[:v_peak] = max(CS[:v_entry], acceleratingCourse[end][:v])
            CS[:v_peak] = v_peak
            CS[:v_exit] = min(CS[:v_exit], CS[:v_peak], acceleratingCourse[end][:v])
        else #CS[:v_entry] == CS[:v_peak]
            # v_exit stays the same
        end #if

        previousCSv_exit = CS[:v_exit]

        # reset the characteristic section (CS), delete behavior sections (BS) that were used during the preperation for setting v_entry, v_peak and v_exit
        CS[:behaviorSections] = Dict()
        CS[:t] = 0.0
    end #for

    return movingSection
end #function secureAcceleratingBehavior!
