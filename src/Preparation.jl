module Preparation

using ..types
include("./MovingPhases.jl")
using .MovingPhases

export preparateSections

## create a moving section and its containing characteristic sections with securedd braking, acceleration and cruising behavior
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

    CSs=[]
    s_csStart=s_entry
    csId=1
    for row in 2:length(path[:sections])
        if min(path[:sections][row-1][:v_limit], v_trainLimit) != min(path[:sections][row][:v_limit], v_trainLimit) || path[:sections][row-1][:f_Rp] != path[:sections][row][:f_Rp]
            push!(CSs, createCharacteristicSection(csId, s_csStart, path[:sections][row-1], min(path[:sections][row-1][:v_limit], v_trainLimit)))
            s_csStart=path[:sections][row][:s_start]
            csId=csId+1
        end #if
    end #for
    push!(CSs, createCharacteristicSection(csId, s_csStart, path[:sections][end], min(path[:sections][end][:v_limit], v_trainLimit)))

    movingSection= Dict(:id => 1,                       # identifier    # if there is more than one moving section in a later version of this tool the id should not be constant anymore
                        :length => pathLength,          # total length (in m)
                        :s_entry => s_entry,            # first position (in m)
                        :s_exit => s_exit,              # last position (in m)
                        :t => 0.0,                      # total running time (in s)
                        :E => 0.0,                      # total energy consumption (in Ws)
                        :t_recovery => 0.0,             # total recovery time for energy-saving modifications (in s)
                        :t_recoveryAvailable => 0.0,    # still available recovery time for energy-saving modifications (in s)
                        :characteristicSections => CSs, # list of containing characteristic sections
                        :energySavingModifications => EnergySavingModification[])   # list of containing all the used energy saving modifications

    return movingSection
end #function createMovingSection


## create a characteristic section for a path section
function createCharacteristicSection(csId::Integer, s_csStart::Real, section::Dict, v_csLimit::Real)
    # this function creates and returns a characteristic section dependent on the paths attributes
    characteristicSection=CharacteristicSection()
    characteristicSection.id=csId                                                               # identifier
    characteristicSection.s_entry=s_csStart                                                     # first position (in m)
    characteristicSection.s_exit=section[:s_end]                                                  # last position  (in m)
    characteristicSection.length=characteristicSection.s_exit-characteristicSection.s_entry     # total length  (in m)
    characteristicSection.t=0.0                                                                 # total running time (in s)
    characteristicSection.E=0.0                                                                 # total energy consumption (in Ws)
    characteristicSection.v_limit=v_csLimit                                                     # speed limit (in m/s)

    # initializing v_entry, v_target and v_exit with v_limit
    characteristicSection.v_target=characteristicSection.v_limit                                # maximum target speed (in m/s)
    characteristicSection.v_entry=characteristicSection.v_limit                                 # maximum entry speed (in m/s)
    characteristicSection.v_exit=characteristicSection.v_limit                                  # maximum exit speed (in m/s)

    characteristicSection.f_Rp=section[:f_Rp] # path resistance (in ‰)

    return characteristicSection
end #function createCharacteristicSection

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(movingSection::Dict, a_braking::Real)
    # this function limits the entry and exit velocity of the characteristic sections to secure that the train stops at the moving sections end
        CSs::Vector{CharacteristicSection} = movingSection[:characteristicSections]

        csId=length(CSs)
        CSs[csId].v_exit=0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            v_entryMax=sqrt(CSs[csId].v_exit^2-2*a_braking*CSs[csId].length)
            v_entryMax=floor(v_entryMax, digits=12)

            CSs[csId].v_entry=min(CSs[csId].v_limit, v_entryMax)
            CSs[csId].v_target=CSs[csId].v_entry
            csId=csId-1
            if csId >= 1
                CSs[csId].v_exit=min(CSs[csId].v_limit, CSs[csId+1].v_entry)
            end #if
        end #while
    return movingSection
end #function secureBrakingBehavior!

## define the intersection velocities between the characterisitc sections to secure acceleration behavior
function secureAccelerationBehavior!(movingSection::Dict, settings::Dict, train::Dict)
    # this function limits the entry and exit velocity of the characteristic sections in case that the train accelerates in every section and cruises aterwards
    CSs::Vector{CharacteristicSection} = movingSection[:characteristicSections]

    CSs[1].v_entry=0.0     # the entry velocity of the first characteristic section is 0.0 m/s
    startingPoint=DataPoint()
    startingPoint.i=1

    previousCSv_exit=CSs[1].v_entry
    for csId in 1:length(CSs)
        CSs[csId].v_entry=min(CSs[csId].v_entry, previousCSv_exit)

        startingPoint.s=CSs[csId].s_entry
        startingPoint.v=CSs[csId].v_entry
        accelerationCourse=[startingPoint]    # List of data points

        if CSs[csId].v_entry<CSs[csId].v_target
            (CSs[csId], accelerationCourse)=addAccelerationPhase!(CSs[csId], accelerationCourse, settings, train, CSs)        # this function changes the accelerationCourse
            CSs[csId].v_target=max(CSs[csId].v_entry,accelerationCourse[end].v)

            CSs[csId].v_exit=min(CSs[csId].v_exit, CSs[csId].v_target, accelerationCourse[end].v)
        else #CSs[csId].v_entry==CSs[csId].v_target
            # v_exit stays the same
        end #if

        previousCSv_exit=CSs[csId].v_exit
    end #for

    return movingSection
end #function secureAccelerationBehavior!



## define the intersection velocities between the characterisitc sections to secure cruising behavior
function secureCruisingBehavior!(movingSection::Dict, settings::Dict, train::Dict)
    # limit the exit velocity of the characteristic sections in case that the train cruises in every section at v_target
    CSs::Vector{CharacteristicSection} = movingSection[:characteristicSections]

    startingPoint=DataPoint()
    startingPoint.i=1

    previousCSv_exit=CSs[1].v_entry

    for csId in 1:length(CSs)
        CSs[csId].v_entry=min(CSs[csId].v_entry, previousCSv_exit)

        startingPoint.s=CSs[csId].s_entry
        startingPoint.v=CSs[csId].v_target
        cruisingCourse=[startingPoint]    # List of data points

        (CSs[csId], cruisingCourse)=addCruisingPhase!(CSs[csId], cruisingCourse, CSs[csId].length, settings, train, CSs, "cruising")        # this function changes the cruisingCourse
        CSs[csId].v_exit=min(CSs[csId].v_exit, cruisingCourse[end].v)

        previousCSv_exit=CSs[csId].v_exit
    end #for

    return movingSection
end #function secureCruisingBehavior!

end #module Preparation
