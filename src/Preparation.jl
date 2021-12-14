module Preparation

using ..types
include("./MovingPhases.jl")
using .MovingPhases

export preparateSections

## create a moving section and its containing characteristic sections with securedd braking, acceleration and cruising behavior
function preparateSections(path::Path, train::Train, settings::Settings)
    movingSection=createMovingSection(path, train.v_limit)
    movingSection=secureBrakingBehavior!(movingSection, train.a_braking)
    movingSection=secureAccelerationBehavior!(movingSection, settings, train)
    movingSection=secureCruisingBehavior!(movingSection, settings, train)
    return movingSection
end #function preparateSections

## create a moving section containing characteristic sections
function createMovingSection(path::Path, v_trainLimit::AbstractFloat)
    # this function creates and returns a moving section dependent on the paths attributes
    movingSection=MovingSection()

    movingSection.id=1          # identifier          # if there is more than one moving section in a later version of this tool the id should not be constant anymore
    movingSection.t=0.0   # total running time (in s)
    movingSection.E=0.0   # total energy consumption (in Ws)

    movingSection.s_entry=path.sections[1].s_start                          # first position (in m)
    movingSection.s_exit=path.sections[length(path.sections)].s_end         # last position (in m)
    movingSection.length=movingSection.s_exit-movingSection.s_entry         # total length (in m)

    s_csStart=movingSection.s_entry
    csId=1
    for row in 2:length(path.sections)
        if min(path.sections[row-1].v_limit, v_trainLimit) != min(path.sections[row].v_limit, v_trainLimit) || path.sections[row-1].f_Rp != path.sections[row].f_Rp
            push!(movingSection.characteristicSections, createCharacteristicSection(csId, s_csStart, path.sections[row-1], min(path.sections[row-1].v_limit, v_trainLimit)))
            s_csStart=path.sections[row].s_start
            csId=csId+1
        end #if
    end #for
    push!(movingSection.characteristicSections, createCharacteristicSection(csId, s_csStart, path.sections[end], min(path.sections[end].v_limit, v_trainLimit)))

    return movingSection
end #function createMovingSection


## create a characteristic section for a path section
function createCharacteristicSection(csId::Integer, s_csStart::AbstractFloat, section::PathSection, v_csLimit::AbstractFloat)
    # this function creates and returns a characteristic section dependent on the paths attributes
    characteristicSection=CharacteristicSection()
    characteristicSection.id=csId                                                               # identifier
    characteristicSection.s_entry=s_csStart                                                     # first position (in m)
    characteristicSection.s_exit=section.s_end                                                  # last position  (in m)
    characteristicSection.length=characteristicSection.s_exit-characteristicSection.s_entry     # total length  (in m)
    characteristicSection.t=0.0                                                                 # total running time (in s)
    characteristicSection.E=0.0                                                                 # total energy consumption (in Ws)
    characteristicSection.v_limit=v_csLimit                                                     # speed limit (in m/s)

    # initializing v_entry, v_target and v_exit with v_limit
    characteristicSection.v_target=characteristicSection.v_limit                                # maximum target speed (in m/s)
    characteristicSection.v_entry=characteristicSection.v_limit                                 # maximum entry speed (in m/s)
    characteristicSection.v_exit=characteristicSection.v_limit                                  # maximum exit speed (in m/s)

    characteristicSection.f_Rp=section.f_Rp # path resistance (in ‰)

    return characteristicSection
end #function createCharacteristicSection

## define the intersection velocities between the characterisitc sections to secure braking behavior
function secureBrakingBehavior!(movingSection::MovingSection, a_braking::AbstractFloat)
    # this function limits the entry and exit velocity of the characteristic sections to secure that the train stops at the moving sections end
        csId=length(movingSection.characteristicSections)
        movingSection.characteristicSections[csId].v_exit=0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            v_entryMax=sqrt(movingSection.characteristicSections[csId].v_exit^2-2*a_braking*movingSection.characteristicSections[csId].length)
            v_entryMax=floor(v_entryMax, digits=12)

            movingSection.characteristicSections[csId].v_entry=min(movingSection.characteristicSections[csId].v_limit, v_entryMax)
            movingSection.characteristicSections[csId].v_target=movingSection.characteristicSections[csId].v_entry
            csId=csId-1
            if csId >= 1
                movingSection.characteristicSections[csId].v_exit=min(movingSection.characteristicSections[csId].v_limit, movingSection.characteristicSections[csId+1].v_entry)
            end #if
        end #while
    return movingSection
end #function secureBrakingBehavior!

## define the intersection velocities between the characterisitc sections to secure acceleration behavior
function secureAccelerationBehavior!(movingSection::MovingSection, settings::Settings, train::Train)
    # this function limits the entry and exit velocity of the characteristic sections in case that the train accelerates in every section and cruises aterwards
    movingSection.characteristicSections[1].v_entry=0.0     # the entry velocity of the first characteristic section is 0.0 m/s
    startingPoint=DataPoint()
    startingPoint.i=1

    previousCSv_exit=movingSection.characteristicSections[1].v_entry
    for csId in 1:length(movingSection.characteristicSections)
        movingSection.characteristicSections[csId].v_entry=min(movingSection.characteristicSections[csId].v_entry, previousCSv_exit)

        startingPoint.s=movingSection.characteristicSections[csId].s_entry
        startingPoint.v=movingSection.characteristicSections[csId].v_entry
        accelerationCourse=[startingPoint]    # List of data points

        if movingSection.characteristicSections[csId].v_entry<movingSection.characteristicSections[csId].v_target
            (movingSection.characteristicSections[csId], accelerationCourse)=addAccelerationPhase!(movingSection.characteristicSections[csId], accelerationCourse, settings, train, movingSection.characteristicSections)        # this function changes the accelerationCourse
            movingSection.characteristicSections[csId].v_target=max(movingSection.characteristicSections[csId].v_entry,accelerationCourse[end].v)

            movingSection.characteristicSections[csId].v_exit=min(movingSection.characteristicSections[csId].v_exit, movingSection.characteristicSections[csId].v_target, accelerationCourse[end].v)
        else #movingSection.characteristicSections[csId].v_entry==movingSection.characteristicSections[csId].v_target
            # v_exit stays the same
        end #if

        previousCSv_exit=movingSection.characteristicSections[csId].v_exit
    end #for

    return movingSection
end #function secureAccelerationBehavior!



## define the intersection velocities between the characterisitc sections to secure cruising behavior
function secureCruisingBehavior!(movingSection::MovingSection, settings::Settings, train::Train)
    # limit the exit velocity of the characteristic sections in case that the train cruises in every section at v_target
    startingPoint=DataPoint()
    startingPoint.i=1

    previousCSv_exit=movingSection.characteristicSections[1].v_entry

    for csId in 1:length(movingSection.characteristicSections)
        movingSection.characteristicSections[csId].v_entry=min(movingSection.characteristicSections[csId].v_entry, previousCSv_exit)

        startingPoint.s=movingSection.characteristicSections[csId].s_entry
        startingPoint.v=movingSection.characteristicSections[csId].v_target
        cruisingCourse=[startingPoint]    # List of data points

        (movingSection.characteristicSections[csId], cruisingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], cruisingCourse, movingSection.characteristicSections[csId].length, settings, train, movingSection.characteristicSections, "cruising")        # this function changes the cruisingCourse
        movingSection.characteristicSections[csId].v_exit=min(movingSection.characteristicSections[csId].v_exit, cruisingCourse[end].v)

        previousCSv_exit=movingSection.characteristicSections[csId].v_exit
    end #for

    return movingSection
end #function secureCruisingBehavior!

end #module Preparation
