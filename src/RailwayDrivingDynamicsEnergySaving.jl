module RailwayDrivingDynamicsEnergySaving

using ..types
using ..RailwayDrivingDynamicsMovingPhases

export calculateRecoveryTime, increaseCostingSection, decreaseMaximumVelocity, combineEnergySavingMethods

function calculateRecoveryTime(s_MS::AbstractFloat, t_MS::AbstractFloat, vehicle::Vehicle)
    # function for calculating the recovery time that can be used for energy saving
    if vehicle.vehicleType=="motor coach train"
        if s_MS<= 30000
            c_s=0.0
        else s_MS> 30000
            c_s=0.0006
        end # if s_MS

        if vehicle.v_limit<=140/3.6     # unit is m/s
            c_t=0.03
        elseif vehicle.v_limit<=160/3.6 # unit is m/s
            c_t=0.04
        elseif vehicle.v_limit<=200/3.6 # unit is m/s
            c_t=0.05
        elseif vehicle.v_limit<=250/3.6 # unit is m/s
            c_t=0.06
        else # vehicle.v_limit>120/3.6  # unit is m/s
            c_t=0.07
        end # if vehicle.v_limit

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    elseif vehicle.vehicleType=="freight" && vehicle.v_limit<=120/3.6   # unit is m/s
            t_recovery1=s_MS*0.0006 +t_MS*0.03
            t_recovery2=s_MS*0.0018 +t_MS*0.0
            t_recovery3=s_MS*0.0 +t_MS*0.04
            t_recovery=max(t_recovery1, t_recovery2, t_recovery3)

            return t_recovery
    else # vehicle.vehicleType=="passenger" || (vehicle.vehicleType=="freight" && vehicle.v_limit>120/3.6) # unit is m/s
        if s_MS<= 30000
            c_s=0.0
        else s_MS> 30000
            c_s=0.0009
        end # if s_MS
        if vehicle.v_limit<=140/3.6         # unit is m/s
            if vehicle.m_union<=300000      # unit is kg
                c_t=0.03
            elseif vehicle.m_union<=500000  # unit is kg
                c_t=0.04
            elseif vehicle.m_union<=700000  # unit is kg
                c_t=0.04
            else # vehicle.m_union>700000   # unit is kg
                c_t=0.05
            end # if vehicle.m_union
        elseif vehicle.v_limit<=160/3.6     # unit is m/s
            if vehicle.m_union<=300000      # unit is kg
                c_t=0.03
            elseif vehicle.m_union<=500000  # unit is kg
                c_t=0.04
            else # vehicle.m_union>500000   # unit is kg
                c_t=0.0
            end # if vehicle.m_union
        elseif vehicle.v_limit<=200/3.6     # unit is m/s
            if vehicle.m_union<=300000      # unit is kg
                c_t=0.04
            elseif vehicle.m_union<=500000  # unit is kg
                c_t=0.05
            else # vehicle.m_union>500000   # unit is kg
                c_t=0.06
            end # if vehicle.m_union
        else # vehicle.v_limit>200/3.6      # unit is m/s
            if vehicle.m_union<=300000      # unit is kg
                c_t=0.05
            elseif vehicle.m_union<=500000  # unit is kg
                c_t=0.06
            else # vehicle.m_union>500000   # unit is kg
                c_t=0.07
            end # if vehicle.m_union
        end # if vehicle.v_limit

        c_tMin=s_MS/t_MS*0.0012
        c_t=max(c_t, c_tMin)

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    end # if vehicle.vehicleType
end #function calculateRecoveryTime


function increaseCostingSection(csOriginal::CharacteristicSection, drivingCourse::Vector{Waypoint}, settings::Settings, vehicle::Vehicle, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "cruising") && haskey(csOriginal.behaviorSections, "braking")
        cruisingReduction=settings.stepSize
        while cruisingReduction>=settings.stepSize/100
            while cruisingReduction>=settings.stepSize/100 # will be done once and repeated twice with smaller cruisingReduction unless !(drivingCourseModified[end].v<=csModified.v_exit && drivingCourseModified[end].s<csModified.s_end) -> see below at the end of the while loop

                # creating a drivingCourse for the characteristic section
                energySavingStartId=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).waypoints[1]
                if energySavingStartId==0
                    error("ERROR at creating a new driving course for energy saving with coasting !")
                end
                drivingCourseModified=[Waypoint(drivingCourse[1])]
                for i in 2:energySavingStartId
                    push!(drivingCourseModified, Waypoint(drivingCourse[i]))    # List of waypoints till the start of energy saving
                end

                # calculating the new length of the cruising section
                if settings.stepVariable=="s in m"                                                           # distance step method
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).s_total-cruisingReduction
                elseif settings.stepVariable=="t in s"                                                       # time step method
                    t_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).t_total-cruisingReduction
                    s_cruising=t_cruising*drivingCourseModified[end].v
                elseif settings.stepVariable=="v in m/s"                                                       # velocity step method
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).s_total-cruisingReduction*10 # TODO: or better: *100 ?
                end #if
                s_cruising=max(0.0, s_cruising)

                # copying csOriginal to csModified
                csModified=CharacteristicSection(csOriginal.id, csOriginal.s_total, csOriginal.s_start, csOriginal.s_end, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_reach, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())
                if haskey(csOriginal.behaviorSections, "starting")
                    startingSection=BehaviorSection(get(csOriginal.behaviorSections, "starting", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("starting"=>startingSection))
                    csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "starting", BehaviorSection()).E_total
                    csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "starting", BehaviorSection()).t_total
                end
                if haskey(csOriginal.behaviorSections, "cruisingBeforeAcceleration")            # this section is needed before acceleration if the mass strip model is used and if the vehicle wants to accelerate to a velocity higher than the limit in the other CS where parts of the union are still located
                    cruisingBeforeAccelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("cruisingBeforeAcceleration"=>cruisingBeforeAccelerationSection))
                    csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()).E_total
                    csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()).t_total
                end
                if haskey(csOriginal.behaviorSections, "acceleration")
                    accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("acceleration"=>accelerationSection))
                    csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "acceleration", BehaviorSection()).E_total
                    csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "acceleration", BehaviorSection()).t_total
                end

                # calculating the new and now shorter cruising section
                if s_cruising>0.0
                    (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, vehicle, allCSs, "cruising")
                end

                (csModified, drivingCourseModified)=addCoastingPhaseWithIntersection!(csModified, drivingCourseModified, settings, vehicle, allCSs)

                if drivingCourseModified[end].v<csModified.v_exit || drivingCourseModified[end].s>csModified.s_end
                    # the vehicle reaches v_exit before reaching s_end. The cruising and coasting sections have to be calculated again with a larger cruising section (so with a smaller reduction of the cruising section)
                    cruisingReduction=cruisingReduction/10
                else
                    break
                end
            end # while cruisingReduction

            if drivingCourseModified[end].v>csModified.v_exit
                (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, vehicle, allCSs)
            elseif drivingCourseModified[end].v==csModified.v_exit && drivingCourseModified[end].s<csModified.s_end
                # v_exit is already reached. Now cruising till the end of the CS
                s_cruisingAfterCoasting=csModified.s_end-drivingCourseModified[end].s
                (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruisingAfterCoasting, settings, vehicle, allCSs, "cruisingAfterCoasting")
            end

            if t_recoveryAvailable<(csModified.t_total-csOriginal.t_total) || drivingCourseModified[end].v!=csModified.v_exit || drivingCourseModified[end].s!=csModified.s_end # time loss is to high and the CS has to be calculated again with larger cruising section (so with a smaller reduction of the cruising section) or v_exit or s_end are not reached excatly
                cruisingReduction=cruisingReduction/10
            else
                return (csModified, drivingCourseModified, true)
            end
        end #while
        # there is no energy saving modification for this CS with the available recovery time
        return (CharacteristicSection(), [], false)
    else
        # there is no energy saving modification for this CS because a cruising section AND a braking section are needed to be transformed into a coasting section
        return (CharacteristicSection(), [], false)
    end
end # function increaseCostingSection



# method 2 with shortening the acceleration by stepsize
function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse, settings::Settings, vehicle::Vehicle, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
 #function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse::Vector{Waypoint}, settings::Settings, vehicle::Vehicle, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "acceleration") && csOriginal.v_reach>csOriginal.v_entry && csOriginal.v_reach>csOriginal.v_exit
                # instead of calculating the new final velocity of the acceleration phase with the step sizes for the different step variables, the velocity of the point before the last will be the new v_reach
            # because it it easier at the moment (TODO)
        lastIdOfOldAccelerationSection=get(csOriginal.behaviorSections, "acceleration", BehaviorSection()).waypoints[end]
        accelerationReduction=drivingCourse[lastIdOfOldAccelerationSection].Δv    # with step size it would be accelerationReduction=settings.stepSize

        while accelerationReduction>=drivingCourse[lastIdOfOldAccelerationSection].Δv/100 # will be done once and repeated twice with smaller accelerationReduction unless t_recoveryAvailable>=(csModified.t_total-csOriginal.t_total) && 0.0<(csModified.t_total-csOriginal.t_total) -> see below at the end of the while loop
            # copying csOriginal to csModified
            csModified=CharacteristicSection(csOriginal.id, csOriginal.s_total, csOriginal.s_start, csOriginal.s_end, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_reach, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())
            if haskey(csOriginal.behaviorSections, "starting")
                startingSection=BehaviorSection(get(csOriginal.behaviorSections, "starting", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("starting"=>startingSection))
                csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "starting", BehaviorSection()).E_total
                csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "starting", BehaviorSection()).t_total
            end

            # creating a drivingCourse for the characteristic section
            energySavingStartId=get(csOriginal.behaviorSections, "cruisingBeforeAcceleration", get(csOriginal.behaviorSections, "acceleration", BehaviorSection())).waypoints[1]

            drivingCourseModified=Vector{Waypoint}()
            for i in 1:energySavingStartId
                push!(drivingCourseModified, Waypoint(drivingCourse[i]))  # List of waypoints till the start of energy saving
            end

            csModified.v_reach=max(csModified.v_entry, csModified.v_exit, csOriginal.v_reach-accelerationReduction)

            # calculation of the new and now shorter acceleration section
            if drivingCourseModified[end].v<csModified.v_reach
                (csModified, drivingCourseModified)=addAccelerationPhase!(csModified, drivingCourseModified, settings, vehicle, allCSs)
            end #if
            # TODO: instead of copying the drivingCourse till the start of acceleration and calculating the acceleration again till the point before the last it would be possible to copy the drivingCouse until this point. Then it would not be necessary to calculate the accelerationPhase again. But then it is difficult to reduce accelerationReduction.

            s_braking=max(0.0, ceil((csModified.v_exit^2-csModified.v_reach^2)/2/vehicle.a_braking, digits=10))       # ceil is used to be sure that the vehicle stops at s_end in spite of rounding errors
            s_cruising=csModified.s_end-drivingCourseModified[end].s-s_braking

            if s_cruising >0.001
                (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, vehicle, allCSs, "cruising")
            end #if

            # s_brakingAfterCruising=ceil((csModified.v_exit^2-drivingCourseModified[end].v^2)/2/vehicle.a_braking, digits=10) #  TODO: check if s_braking and s_brakingAfterCruising are really always the same
            if drivingCourseModified[end].v>csModified.v_exit
                (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, vehicle, allCSs)
            elseif drivingCourseModified[end].s<csModified.s_end
                if (csModified.s_end-drivingCourseModified[end].s)>0.001
                    println("WARNING: The end of new CS",csModified.id," is not reached while saving energy with lowering v_reach.")
                    println("         Therefore   s=",drivingCourseModified[end].s," will be set s_end=",csModified.s_end," because the difference is only ",csModified.s_end-drivingCourseModified[end].s," m.")
                end
                drivingCourseModified[end].s=csModified.s_end       # rounding up to s_end
            end #if

            if t_recoveryAvailable>=(csModified.t_total-csOriginal.t_total)
                return (csModified, drivingCourseModified, true)
            else # time loss is to high and the CS has to be calculated again with larger acceleration section (so with a smaller reduction of the acceleration section)
                accelerationReduction=min(accelerationReduction/10, csModified.v_reach-csModified.v_entry, csModified.v_reach-csModified.v_exit)
            end
        end #while

        # there is no energy saving modification for this CS with the available recovery time
        return (CharacteristicSection(), [], false)

    else
        # there is no energy saving modification for this CS because v_reach can not be lowered below v_entry or v_exit or because there is no acceleration section that can be transformed into a cruising section
        return (CharacteristicSection(), [], false)
    end #if haskey
end # function decreaseMaximumVelocity

# combination of method 1 and method 2
function combineEnergySavingMethods(csOriginal::CharacteristicSection, drivingCourse::Vector{Waypoint}, settings::Settings, vehicle::Vehicle, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "acceleration") && (haskey(csOriginal.behaviorSections, "braking") || haskey(csOriginal.behaviorSections, "coasting")) && csOriginal.v_reach>csOriginal.v_entry && csOriginal.v_reach>csOriginal.v_exit
        csCombined=CharacteristicSection(csOriginal)
        drivingCourseCombined=Vector{Waypoint}()
        for i in 1:length(drivingCourse)
            push!(drivingCourseCombined, Waypoint(drivingCourse[i]))
        end
        ΔE=0.0      # saved energy (in Ws)
        Δt=0.0      # time loss (in s)
        while (Δt<t_recoveryAvailable && ΔE<=0.0)
            (csCombined, drivingCourseCombined, newMaximumVelocity)=decreaseMaximumVelocity(csCombined, drivingCourseCombined, settings, vehicle, allCSs, t_recoveryAvailable)
            if newMaximumVelocity
                ΔE=csOriginal.E_total-csCombined.E_total      # saved energy (in Ws)
                Δt=csCombined.t_total-csOriginal.t_total      # time loss (in s)

                while (haskey(csOriginal.behaviorSections, "cruising") && (Δt<=0.0 || ΔE<=0.0)) #Δt<=0.0) #&& ΔE<=0.0) # && Δt<=0.0)
                    (csCombined, drivingCourseCombined, newCoasting)=increaseCostingSection(csCombined, drivingCourseCombined, settings, vehicle, allCSs, t_recoveryAvailable-Δt)
                    if newCoasting
                        ΔE=csOriginal.E_total-csCombined.E_total      # saved energy (in Ws)
                        Δt=csCombined.t_total-csOriginal.t_total      # time loss (in s)
                    else
                        break
                    end # if newCoasting
                end # while haskey
            else
                return (CharacteristicSection(), [], false)
            end # if newMaximumVelocity
        end # while
        return (csCombined, drivingCourseCombined, (ΔE>0.0))# && Δt>0.0))
    else
        # there is no energy saving modification for this CS because v_reach can not be lowered below v_entry or v_exit or because there is no acceleration section and braking section or coasting section that can be transformed into a cruising section or coasting section
        return (CharacteristicSection(), [], false)
    end #if
end #function combineEnergySavingMethods

end #module RailwayDrivingDynamicsEnergySaving
