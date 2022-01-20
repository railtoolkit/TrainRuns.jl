module Preparation

include("./Behavior.jl")
using .Behavior

export preparateSections

## create a moving section and its containing characteristic sections with secured braking, acceleration and cruising behavior
function preparateSections(path::Dict, train::Dict, settings::Dict)
    movingSection=createMovingSection(path, train[:v_limit])
    movingSection=secureBrakingBehavior!(movingSection, train[:a_braking])
    movingSection=secureAccelerationBehavior!(movingSection, settings, train)
    movingSection=secureCruisingBehavior!(movingSection, settings, train)
    return movingSection
end #function preparateSections

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
        if min(previousSection[:v_limit], v_trainLimit) != min(currentSection[:v_limit], v_trainLimit) || previousSection[:f_Rp] != currentSection[:f_Rp]
            push!(CSs, createCharacteristicSection(csId, s_csStart, previousSection, min(previousSection[:v_limit], v_trainLimit)))
            s_csStart = currentSection[:s_start]
            csId = csId+1
        end #if
    end #for
    push!(CSs, createCharacteristicSection(csId, s_csStart, path[:sections][end], min(path[:sections][end][:v_limit], v_trainLimit)))

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
function createCharacteristicSection(csId::Integer, s_csStart::Real, section::Dict, v_csLimit::Real)
    # Create and return a characteristic section dependent on the paths attributes
    characteristicSection= Dict(:id => csId,                            # identifier
                                :s_entry => s_csStart,                  # first position (in m)
                                :s_exit => section[:s_end],             # last position  (in m)
                                :length => section[:s_end]-s_csStart,   # total length  (in m)
                                :r_path => section[:f_Rp],              # path resistance (in ‰)
                                :behaviorSections => Dict(),            # list of containing behavior sections
                                :t => 0.0,                              # total running time (in s)
                                :E => 0.0,                              # total energy consumption (in Ws)
                                :v_limit => v_csLimit,                  # speed limit (in m/s)
                                # initializing :v_entry, :v_peak and :v_exit with :v_limit
                                :v_peak => v_csLimit,                   # maximum reachable speed (in m/s)
                                :v_entry => v_csLimit,                  # maximum entry speed (in m/s)
                                :v_exit => v_csLimit)                   # maximum exit speed (in m/s)
    return characteristicSection
end #function createCharacteristicSection

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(movingSection::Dict, a_braking::Real)
    # this function limits the entry and exit velocity of the characteristic sections to secure that the train stops at the moving sections end
        CSs = movingSection[:characteristicSections]

        csId = length(CSs)
        CSs[csId][:v_exit] = 0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            v_entryMax = calcBrakingStartVelocity(CSs[csId][:v_exit], a_braking, CSs[csId][:length])
            #v_entryMax=floor(v_entryMax, digits=12)

            CSs[csId][:v_entry] = min(CSs[csId][:v_limit], v_entryMax)
            CSs[csId][:v_peak] = CSs[csId][:v_entry]
            csId = csId - 1
            if csId >= 1
                CSs[csId][:v_exit]=min(CSs[csId][:v_limit], CSs[csId+1][:v_entry])
            end #if
        end #while
    return movingSection
end #function secureBrakingBehavior!

## define the intersection velocities between the characterisitc sections to secure acceleration behavior
function secureAccelerationBehavior!(movingSection::Dict, settings::Dict, train::Dict)
    # this function limits the entry and exit velocity of the characteristic sections in case that the train accelerates in every section and cruises aterwards
    CSs = movingSection[:characteristicSections]

    CSs[1][:v_entry] = 0.0     # the entry velocity of the first characteristic section is 0.0 m/s
    startingPoint = createDataPoint()
    startingPoint[:i] = 1

    previousCSv_exit = CSs[1][:v_entry]
    for csId in 1:length(CSs)
        CSs[csId][:v_entry] = min(CSs[csId][:v_entry], previousCSv_exit)

        startingPoint[:s] = CSs[csId][:s_entry]
        startingPoint[:v] = CSs[csId][:v_entry]
        accelerationCourse::Vector{Dict} = [startingPoint]    # List of data points

        if CSs[csId][:v_entry] < CSs[csId][:v_peak]
            (CSs[csId], accelerationCourse) = addAccelerationSection!(CSs[csId], accelerationCourse, settings, train, CSs)        # this function changes the accelerationCourse
            CSs[csId][:v_peak] = max(CSs[csId][:v_entry], accelerationCourse[end][:v])
            CSs[csId][:v_exit] = min(CSs[csId][:v_exit], CSs[csId][:v_peak], accelerationCourse[end][:v])
        else #CSs[csId][:v_entry]==CSs[csId][:v_peak]
            # v_exit stays the same
        end #if

        previousCSv_exit=CSs[csId][:v_exit]
    end #for

    return movingSection
end #function secureAccelerationBehavior!



## define the intersection velocities between the characterisitc sections to secure cruising behavior
function secureCruisingBehavior!(movingSection::Dict, settings::Dict, train::Dict)
    # limit the exit velocity of the characteristic sections in case that the train cruises in every section at v_peak
    CSs = movingSection[:characteristicSections]

    startingPoint = createDataPoint()
    startingPoint[:i] = 1

    previousCSv_exit = CSs[1][:v_entry]

    for csId in 1:length(CSs)
        CSs[csId][:v_entry] = min(CSs[csId][:v_entry], previousCSv_exit)

        startingPoint[:s] = CSs[csId][:s_entry]
        startingPoint[:v] = CSs[csId][:v_peak]
        cruisingCourse::Vector{Dict} = [startingPoint]    # List of data points

        (CSs[csId], cruisingCourse) = addCruisingSection!(CSs[csId], cruisingCourse, CSs[csId][:length], settings, train, CSs, "cruising")        # this function changes the cruisingCourse
        CSs[csId][:v_exit] = min(CSs[csId][:v_exit], cruisingCourse[end][:v])

        previousCSv_exit = CSs[csId][:v_exit]
    end #for

    return movingSection
end #function secureCruisingBehavior!

end #module Preparation
