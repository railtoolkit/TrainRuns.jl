module RailwayDrivingDynamicsPreparation

using ..types
include("./RailwayDrivingDynamicsMovingPhases.jl")
using .RailwayDrivingDynamicsMovingPhases

export preparateSections

## creating a moving section and its containing characteristic sections which braking and acceleration behavior is secured
function preparateSections(path::Path, vehicle::Vehicle, settings::Settings)
    movingSection=createMovingSection(path, vehicle.v_limit)
    movingSection=secureBrakingBehavior!(movingSection, vehicle.a_braking)
    movingSection=secureAccelerationBehavior!(movingSection, settings, vehicle)
    return movingSection
end #function preparateSections

## creating a moving section containing characteristic sections
function createMovingSection(path::Path, v_vehicleLimit::AbstractFloat)
    # this function creates and returns a moving section dependent on the paths attributes
    movingSection=MovingSection()

    movingSection.id=1          # identifier          # if there is more than one moving section in a later version of this tool the id should not be constant anymore
    movingSection.t_total=0.0   # total running time (in s)
    movingSection.E_total=0.0   # total energy consumption (in Ws)

    movingSection.s_start=path.sections[1].s_start                          # first position (in m)
    movingSection.s_end=path.sections[length(path.sections)].s_startNext    # last position (in m)
    movingSection.s_total=movingSection.s_end-movingSection.s_start         # total length (in m)

    s_csStart=movingSection.s_start
    csId=1
    for row in 2:length(path.sections)
        if min(path.sections[row-1].v_limit, v_vehicleLimit) != min(path.sections[row].v_limit, v_vehicleLimit) || path.sections[row-1].f_Rp != path.sections[row].f_Rp
            push!(movingSection.characteristicSections, createCharacteristicSection(csId, s_csStart, path.sections[row-1], min(path.sections[row-1].v_limit, v_vehicleLimit)))
            s_csStart=path.sections[row].s_start
            csId=csId+1
        end #if
    end #for
    push!(movingSection.characteristicSections, createCharacteristicSection(csId, s_csStart, path.sections[end], min(path.sections[end].v_limit, v_vehicleLimit)))

    return movingSection
end #function createMovingSection


##
function createCharacteristicSection(csId::Integer, s_csStart::AbstractFloat, section::PathSection, v_csLimit::AbstractFloat)
    # this function creates and returns a characteristic section dependent on the paths attributes
    characteristicSection=CharacteristicSection()
    characteristicSection.id=csId                                                               # identifier
    characteristicSection.s_start=s_csStart                                                     # first position (in m)
    characteristicSection.s_end=section.s_startNext                                             # last position  (in m)
    characteristicSection.s_total=characteristicSection.s_end-characteristicSection.s_start     # total length  (in m)
    characteristicSection.t_total=0.0                                                           # total running time (in s)
    characteristicSection.E_total=0.0                                                           # total energy consumption (in Ws)
    characteristicSection.v_limit=v_csLimit                                                     # speed limit (in m/s)

    # initializing v_entry, v_reach and v_exit with v_limit
    characteristicSection.v_reach=characteristicSection.v_limit                                 # maximum reachable speed (in m/s)
    characteristicSection.v_entry=characteristicSection.v_limit                                 # maximum entry speed (in m/s)
    characteristicSection.v_exit=characteristicSection.v_limit                                  # maximum exit speed (in m/s)

    characteristicSection.f_Rp=section.f_Rp # path resistance (in ‰)

    return characteristicSection
end #function createCharacteristicSection

## defining the intersection velocities between the characterisitc sections
function secureBrakingBehavior!(movingSection::MovingSection, a_braking::AbstractFloat)
    # this function limits the entry and exit velocity of the characteristic sections to secure that the vehicle stops at the moving sections end
        csId=length(movingSection.characteristicSections)
        movingSection.characteristicSections[csId].v_exit=0.0     # the exit velocity of the last characteristic section is 0.0 m/s
        while csId >= 1
            v_entryMax=sqrt(movingSection.characteristicSections[csId].v_exit^2-2*a_braking*movingSection.characteristicSections[csId].s_total)
            v_entryMax=floor(v_entryMax, digits=12)

            movingSection.characteristicSections[csId].v_entry=min(movingSection.characteristicSections[csId].v_limit, v_entryMax)
            movingSection.characteristicSections[csId].v_reach=movingSection.characteristicSections[csId].v_entry
            csId=csId-1
            if csId >= 1
                movingSection.characteristicSections[csId].v_exit=min(movingSection.characteristicSections[csId].v_limit, movingSection.characteristicSections[csId+1].v_entry)
            end #if
        end #while
    return movingSection
end #function secureBrakingBehavior!

##
function secureAccelerationBehavior!(movingSection::MovingSection, settings::Settings, vehicle::Vehicle)
    # this function limits the entry and exit velocity of the characteristic sections for case that the vehicle accelerates in every section and cruises aterwards
    movingSection.characteristicSections[1].v_entry=0.0     # the entry velocity of the first characteristic section is 0.0 m/s
    startingPoint=Waypoint()
    startingPoint.i=1

    previousCSv_exit=0.0

    for csId in 1:length(movingSection.characteristicSections)
        movingSection.characteristicSections[csId].v_entry=min(movingSection.characteristicSections[csId].v_entry, previousCSv_exit)

        startingPoint.s=movingSection.characteristicSections[csId].s_start
        startingPoint.v=movingSection.characteristicSections[csId].v_entry
        accelerationCourse=[startingPoint]    # List of waypoints


        if movingSection.characteristicSections[csId].v_entry<movingSection.characteristicSections[csId].v_reach
            (movingSection.characteristicSections[csId], accelerationCourse)=addAccelerationPhase!(movingSection.characteristicSections[csId], accelerationCourse, settings, vehicle, movingSection.characteristicSections)        # this function changes the accelerationCourse
            movingSection.characteristicSections[csId].v_reach=accelerationCourse[end].v
            movingSection.characteristicSections[csId].v_exit=min(movingSection.characteristicSections[csId].v_exit, movingSection.characteristicSections[csId].v_reach)

            if settings.stepVariable=="v in m/s"   # with the new v_reach the calculation will be defferent a second time. Therefore the CS has to be calculated again for velocity step method to refresh the length of the acceleration section for further simulation
                delete!(movingSection.characteristicSections[csId].behaviorSections, "starting")
                delete!(movingSection.characteristicSections[csId].behaviorSections, "cruisingBeforeAcceleration")
                delete!(movingSection.characteristicSections[csId].behaviorSections, "acceleration")
                movingSection.characteristicSections[csId].E_total=0.0
                movingSection.characteristicSections[csId].t_total=0.0
                accelerationCourse=[startingPoint]    # List of waypoints

                (movingSection.characteristicSections[csId], accelerationCourse)=addAccelerationPhase!(movingSection.characteristicSections[csId], accelerationCourse, settings, vehicle, movingSection.characteristicSections)        # this function changes the accelerationCourse
            end
        else #movingSection.characteristicSections[csId].v_entry==movingSection.characteristicSections[csId].v_reach
                        # v_exit stays the same
        end #if

        # securing cruising behavior
        # is need if the path resistance is too high, the vehicle can not cruise at v_reach and is getting slower
        s_cruisingMax=movingSection.characteristicSections[csId].s_end-accelerationCourse[end].s
        if s_cruisingMax>0.0
            (movingSection.characteristicSections[csId], accelerationCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], accelerationCourse, s_cruisingMax, settings, vehicle, movingSection.characteristicSections, "cruising")
            movingSection.characteristicSections[csId].v_exit=min(movingSection.characteristicSections[csId].v_exit, accelerationCourse[end].v)
        end

        previousCSv_exit=movingSection.characteristicSections[csId].v_exit
    #    println("CS",csId,":  s_start: ",movingSection.characteristicSections[csId].s_start," v_entry: ", round(movingSection.characteristicSections[csId].v_entry*3.6, digits=5), ", v_reach: ",  round(movingSection.characteristicSections[csId].v_reach*3.6, digits=5), "  v_exit: ", round(movingSection.characteristicSections[csId].v_exit*3.6, digits=5))
    end #for

    return movingSection
end #function secureAccelerationBehavior!

end #module RailwayDrivingDynamicsPreparation
