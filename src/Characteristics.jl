#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

## create a moving section and its containing characteristic sections with secured braking, accelerating and cruising behavior
function determineCharacteristics(path::Dict, train::Dict, settings::Settings)
    movingSection = createMovingSection(path, train[:v_limit])
    movingSection = secureBrakingBehavior!(movingSection, train[:a_braking])
    movingSection = secureAcceleratingBehavior!(movingSection, settings, train)
    #movingSection = secureCruisingBehavior!(movingSection, settings, train)

    return movingSection
end #function determineCharacteristics

## create a moving section containing characteristic sections
function createMovingSection(path::Dict, v_trainLimit::Real)
    # this function creates and returns a moving section dependent on the paths attributes

    s_entry = path[:sections][1][:s_start]          # first position (in m)
    s_exit = path[:sections][end][:s_end]           # last position (in m)
    pathLength = s_exit - s_entry                   # total length (in m)

    CSs=Vector{Dict}()
    s_csStart=s_entry
    csId=1
    for row in 2:length(path[:sections])
        previousSection = path[:sections][row-1]
        currentSection = path[:sections][row]
        speedLimitIsDifferent = min(previousSection[:v_limit], v_trainLimit) != min(currentSection[:v_limit], v_trainLimit)
        pathResistanceIsDifferent = previousSection[:f_Rp] != currentSection[:f_Rp]
        if speedLimitIsDifferent || pathResistanceIsDifferent
        # 03/09 old: if min(previousSection[:v_limit], v_trainLimit) != min(currentSection[:v_limit], v_trainLimit) || previousSection[:f_Rp] != currentSection[:f_Rp]
            push!(CSs, createCharacteristicSection(csId, s_csStart, previousSection, min(previousSection[:v_limit], v_trainLimit), path))
            s_csStart = currentSection[:s_start]
            csId = csId+1
        end #if
    end #for
    push!(CSs, createCharacteristicSection(csId, s_csStart, path[:sections][end], min(path[:sections][end][:v_limit], v_trainLimit), path))

    movingSection= Dict(:id => 1,                       # identifier    # if there is more than one moving section in a later version of this tool the id should not be constant anymore
                        :length => pathLength,          # total length (in m)
                        :s_entry => s_entry,            # first position (in m)
                        :s_exit => s_exit,              # last position (in m)
                        :t => 0.0,                      # total running time (in s)
                        :E => 0.0,                      # total energy consumption (in Ws)
                        :characteristicSections => CSs) # list of containing characteristic sections

    return movingSection
end #function createMovingSection


## create a characteristic section for a path section. A characteristic section is a part of the moving section. It contains behavior sections.
function createCharacteristicSection(id::Integer, s_entry::Real, section::Dict, v_limit::Real, path::Dict)
    # Create and return a characteristic section dependent on the paths attributes
    characteristicSection= Dict(:id => id,                            # identifier
                                :s_entry => s_entry,                    # first position (in m)
                                :s_exit => section[:s_end],             # last position  (in m)
                                :length => section[:s_end] -s_entry,    # total length  (in m)
                                :r_path => section[:f_Rp],              # path resistance (in ‰)
                                :behaviorSections => Dict(),            # list of containing behavior sections
                                :t => 0.0,                              # total running time (in s)
                                :E => 0.0,                              # total energy consumption (in Ws)
                                :v_limit => v_limit,                    # speed limit (in m/s)
                                # initializing :v_entry, :v_peak and :v_exit with :v_limit
                                :v_peak => v_limit,                     # maximum reachable speed (in m/s)
                                :v_entry => v_limit,                    # maximum entry speed (in m/s)
                                :v_exit => v_limit)                     # maximum exit speed (in m/s)

    # list of positions of every point of interest (POI) in this charateristic section for which data points should be calculated
    s_exit = characteristicSection[:s_exit]
    pointsOfInterest = Vector{Real}()
    if haskey(path, :pointsOfInterest)
        for POI in path[:pointsOfInterest]
            if s_entry < POI && POI < s_exit
                push!(pointsOfInterest, POI)
            end
        end
    end
    push!(pointsOfInterest, s_exit)     # s_exit has to be the last POI so that there will always be a POI to campare the current position with

    merge!(characteristicSection, Dict(:pointsOfInterest => pointsOfInterest))

    return characteristicSection
end #function createCharacteristicSection

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(movingSection::Dict, a_braking::Real)
    # this function limits the entry and exit velocity of the characteristic sections to secure that the train stops at the moving sections end
        CSs = movingSection[:characteristicSections]

        csId = length(CSs)
        followingCSv_entry = 0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            CS = CSs[csId]

            CS[:v_exit] = min(CS[:v_limit], followingCSv_entry)

            v_entryMax = calcBrakingStartVelocity(CS[:v_exit], a_braking, CS[:length])

            CS[:v_entry] = min(CS[:v_limit], v_entryMax)
            CS[:v_peak] = CS[:v_entry]


            # reset the characteristic section (CS), delete behavior sections (BS) that were used during the preperation for setting v_entry, v_peak and v_exit
            CS[:behaviorSections] = Dict()
            CS[:E] = 0.0
            CS[:t] = 0.0

            followingCSv_entry = CS[:v_entry]
            csId = csId - 1
        end #while
    return movingSection
end #function secureBrakingBehavior!

## define the intersection velocities between the characterisitc sections to secure accelerating behavior
function secureAcceleratingBehavior!(movingSection::Dict, settings::Settings, train::Dict)
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
                    if settings.massModel == :mass_point || acceleratingCourse[end][:s] > CS[:s_entry] + train[:length]
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
        CS[:E] = 0.0
        CS[:t] = 0.0
    end #for

    return movingSection
end #function secureAcceleratingBehavior!


#=
## define the intersection velocities between the characterisitc sections to secure cruising behavior
function secureCruisingBehavior!(movingSection::Dict, settings::Settings, train::Dict)
    # limit the exit velocity of the characteristic sections in case that the train cruises in every section at v_peak
    CSs = movingSection[:characteristicSections]

    startingPoint = createDataPoint()
    startingPoint[:i] = 1

    previousCSv_exit = CSs[1][:v_entry]

    for CS in CSs
        # conditions for entering the cruising phase
        stateFlags = Dict(:endOfCSReached => false,
                          :brakingStartReached => false,
                          :tractionDeficit => false,
                          :resistingForceNegative => false,
                          :previousSpeedLimitReached => false,
                          :speedLimitReached => false,
                          :error => false,
                          :usedForDefiningCharacteristics => true)

        CS[:v_entry] = min(CS[:v_entry], previousCSv_exit)

        startingPoint[:s] = CS[:s_entry]
        startingPoint[:v] = CS[:v_peak]
        cruisingCourse::Vector{Dict} = [startingPoint]    # List of data points

        while !stateFlags[:endOfCSReached] #&& s_cruising > 0.0
            if !stateFlags[:tractionDeficit]
                s_cruising = CS[:s_exit] - cruisingCourse[end][:s]
                if !stateFlags[:resistingForceNegative]# cruisingCourse[end][:F_R] >= 0
                    (CS, cruisingCourse, stateFlags) = addCruisingSection!(CS, cruisingCourse, stateFlags, s_cruising, settings, train, CSs, "cruising")        # this function changes the cruisingCourse
                else
                    (CS, cruisingCourse, stateFlags) = addCruisingSection!(CS, cruisingCourse, stateFlags, s_cruising, settings, train, CSs, "downhillBraking")
                end
            else
                if settings.massModel == :mass_point || cruisingCourse[end][:s] > CS[:s_entry] + train[:length]
                    break
                else
                    (CS, cruisingCourse, stateFlags) = addDiminishingSection!(CS, cruisingCourse, stateFlags, settings, train, CSs)        # this function is needed in case the resisitng forces are higher than the maximum possible tractive effort
                end
            end
        end

        CS[:v_exit] = min(CS[:v_exit], cruisingCourse[end][:v])

        previousCSv_exit = CS[:v_exit]

        # reset the characteristic section (CS), delete behavior sections (BS) that were used during the preperation for setting v_entry, v_peak and v_exit
        CS[:behaviorSections] = Dict()
        CS[:E] = 0.0
        CS[:t] = 0.0
    end #for

    return movingSection
end #function secureCruisingBehavior!
=#
