module EnergySaving

using ..types
using ..MovingPhases

export calculateRecoveryTime, increaseCoastingSection, decreaseMaximumVelocity, combineEnergySavingMethods

approximationLevel = 6  # value for approximation to intersections
    # TODO: define it in TrainRun and give it to each function?

function calculateRecoveryTime(s_MS::AbstractFloat, t_MS::AbstractFloat, train::Train)
    # function for calculating the recovery time that can be used for energy saving
    # MS: Moving Section
    if train.trainType=="motor coach train"
        if s_MS<= 30000
            c_s=0.0
        else s_MS> 30000
            c_s=0.0006
        end # if s_MS

        if train.v_limit<=140/3.6     # unit is m/s
            c_t=0.03
        elseif train.v_limit<=160/3.6 # unit is m/s
            c_t=0.04
        elseif train.v_limit<=200/3.6 # unit is m/s
            c_t=0.05
        elseif train.v_limit<=250/3.6 # unit is m/s
            c_t=0.06
        else # train.v_limit>120/3.6  # unit is m/s
            c_t=0.07
        end # if train.v_limit

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    elseif train.trainType=="freight" && train.v_limit<=120/3.6   # unit is m/s
            t_recovery1=s_MS*0.0006 +t_MS*0.03
            t_recovery2=s_MS*0.0018 +t_MS*0.0
            t_recovery3=s_MS*0.0 +t_MS*0.04
            t_recovery=max(t_recovery1, t_recovery2, t_recovery3)

            return t_recovery
    else # train.trainType=="passenger" || (train.trainType=="freight" && train.v_limit>120/3.6) # unit is m/s
        if s_MS<= 30000
            c_s=0.0
        else s_MS> 30000
            c_s=0.0009
        end # if s_MS
        if train.v_limit<=140/3.6         # unit is m/s
            if train.m_union<=300000      # unit is kg
                c_t=0.03
            elseif train.m_union<=500000  # unit is kg
                c_t=0.04
            elseif train.m_union<=700000  # unit is kg
                c_t=0.04
            else # train.m_union>700000   # unit is kg
                c_t=0.05
            end # if train.m_union
        elseif train.v_limit<=160/3.6     # unit is m/s
            if train.m_union<=300000      # unit is kg
                c_t=0.03
            elseif train.m_union<=500000  # unit is kg
                c_t=0.04
            else # train.m_union>500000   # unit is kg
                c_t=0.0
            end # if train.m_union
        elseif train.v_limit<=200/3.6     # unit is m/s
            if train.m_union<=300000      # unit is kg
                c_t=0.04
            elseif train.m_union<=500000  # unit is kg
                c_t=0.05
            else # train.m_union>500000   # unit is kg
                c_t=0.06
            end # if train.m_union
        else # train.v_limit>200/3.6      # unit is m/s
            if train.m_union<=300000      # unit is kg
                c_t=0.05
            elseif train.m_union<=500000  # unit is kg
                c_t=0.06
            else # train.m_union>500000   # unit is kg
                c_t=0.07
            end # if train.m_union
        end # if train.v_limit

        c_tMin=s_MS/t_MS*0.0012
        c_t=max(c_t, c_tMin)

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    end # if train.trainType
end #function calculateRecoveryTime

function increaseCoastingSection(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "cruising") && haskey(csOriginal.behaviorSections, "braking")
        cruisingReduction=settings.stepSize
        while cruisingReduction>=settings.stepSize/10^approximationLevel
        #while cruisingReduction>=settings.stepSize/100
            while cruisingReduction>=settings.stepSize/10^approximationLevel # will be done once and then depending on approximationLevel repeated with smaller cruisingReduction unless !(drivingCourseModified[end].v<=csModified.v_exit && drivingCourseModified[end].s<csModified.s_end) -> see below at the end of the while loop

                # create a copy for the characteristic sections drivingCourse
                energySavingStartId=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[1]
                if energySavingStartId==0
                    error("ERROR at creating a new driving course for energy saving with coasting !")
                end
                    # TODO: tried to insert copy on 15.07.2021 drivingCourseModified=[copy(drivingCourse[1])]
                drivingCourseModified=[DataPoint(drivingCourse[1])]
                for i in 2:energySavingStartId
                        # TODO: tried to insert copy on 15.07.2021 push!(drivingCourseModified, copy(drivingCourse[i]))    # List of data points till the start of energy saving
                    push!(drivingCourseModified, DataPoint(drivingCourse[i]))    # List of data points till the start of energy saving
                end

                # calculating the new length of the cruising section
                if settings.stepVariable=="s in m"                                                           # distance step method
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).s_total-cruisingReduction
                elseif settings.stepVariable=="t in s"                                                       # time step method
                    # 09/20 old: doesn't work for non constant cruising
                        # t_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).t_total-cruisingReduction
                        # s_cruising=t_cruising*drivingCourseModified[end].v
                    wayReduction=drivingCourse(get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[end]).v*cruisingReduction
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).s_total-wayReduction

                elseif settings.stepVariable=="v in m/s"                                                       # velocity step method
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).s_total-cruisingReduction*10 # TODO: or better: *100 ?
                end #if
                s_cruising=max(0.0, s_cruising)

                # copy csOriginal to csModified
                csModified=CharacteristicSection(csOriginal.id, csOriginal.s_total, csOriginal.s_start, csOriginal.s_end, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_reach, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())
                if haskey(csOriginal.behaviorSections, "starting")
                    startingSection=BehaviorSection(get(csOriginal.behaviorSections, "starting", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("starting"=>startingSection))
                    csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "starting", BehaviorSection()).E_total
                    csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "starting", BehaviorSection()).t_total
                end
                if haskey(csOriginal.behaviorSections, "cruisingBeforeAcceleration")            # this section is needed before acceleration if the mass strip model is used and if the train wants to accelerate to a velocity higher than the limit in the other CS where parts of the union are still located
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

                # simulate the new and now shorter cruising section
                if s_cruising>0.0
                    (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
                end

                (csModified, drivingCourseModified)=addCoastingPhaseUntilBraking!(csModified, drivingCourseModified, settings, train, allCSs)

                if drivingCourseModified[end].v < csModified.v_exit || drivingCourseModified[end].s > csModified.s_end
                    # the train reaches v_exit before reaching s_end. The cruising and coasting sections have to be calculated again with a larger cruising section (so with a smaller reduction of the cruising section)
                    cruisingReduction=cruisingReduction/10
                else
                    break
                end
            end # while cruisingReduction

            if drivingCourseModified[end].v > csModified.v_exit
                #(csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, train, allCSs)
                (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings, train, allCSs)
            elseif drivingCourseModified[end].v == csModified.v_exit && drivingCourseModified[end].s < csModified.s_end
                # v_exit is already reached. Now cruise till the end of the CS
                s_cruisingAfterCoasting=csModified.s_end-drivingCourseModified[end].s
                (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruisingAfterCoasting, settings, train, allCSs, "cruisingAfterCoasting")
            end

            if t_recoveryAvailable < csModified.t_total-csOriginal.t_total || drivingCourseModified[end].v != csModified.v_exit || drivingCourseModified[end].s != csModified.s_end # time loss is to high and the CS has to be calculated again with larger cruising section (so with a smaller reduction of the cruising section) or v_exit or s_end are not reached excatly
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
end # function increaseCoastingSection


# method 2 with shortening the acceleration by stepsize
function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
 #function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "acceleration") && csOriginal.v_reach > csOriginal.v_entry && csOriginal.v_reach > csOriginal.v_exit
        accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))
        if drivingCourse[accelerationSection.dataPoints[end]-1].v < csOriginal.v_exit
            return (CharacteristicSection(), [], false)
            # TODO: or calculate a new acceleration phase with v_exit as v_reach? it will be very short, shorter than the step size.
        end

        # copy csOriginal to csModified
        csModified=CharacteristicSection(csOriginal.id, csOriginal.s_total, csOriginal.s_start, csOriginal.s_end, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_reach, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())

        if haskey(csOriginal.behaviorSections, "starting")
            startingSection=BehaviorSection(get(csOriginal.behaviorSections, "starting", BehaviorSection()))
            merge!(csModified.behaviorSections, Dict("starting"=>startingSection))
            csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "starting", BehaviorSection()).E_total
            csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "starting", BehaviorSection()).t_total
        end

        #accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))

        if length(accelerationSection.dataPoints) > 2
            if haskey(csOriginal.behaviorSections, "cruisingBeforeAcceleration")
                cruisingBeforeAccelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("cruisingBeforeAcceleration"=>cruisingBeforeAccelerationSection))
                csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()).E_total
                csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()).t_total
            end

            # remove the last acceleration waypoint
            pop!(accelerationSection.dataPoints)

            accelerationSection.v_exit=drivingCourse[accelerationSection.dataPoints[end]].v                               # exit speed (in m/s)
            accelerationSection.s_end=drivingCourse[accelerationSection.dataPoints[end]].s                                # last position (in m)
            accelerationSection.s_total=accelerationSection.s_end-accelerationSection.s_start           # total length  (in m)
            accelerationSection.t_total=drivingCourse[accelerationSection.dataPoints[end]].t-drivingCourse[accelerationSection.dataPoints[1]].t       # total running time (in s)
            accelerationSection.E_total=drivingCourse[accelerationSection.dataPoints[end]].E-drivingCourse[accelerationSection.dataPoints[1]].E       # total energy consumption (in Ws)

            merge!(csModified.behaviorSections, Dict("acceleration"=>accelerationSection))
            csModified.E_total=csModified.E_total+get(csModified.behaviorSections, "acceleration", BehaviorSection()).E_total
            csModified.t_total=csModified.t_total+get(csModified.behaviorSections, "acceleration", BehaviorSection()).t_total

            energySavingStartId=accelerationSection.dataPoints[end]
        else
            # The acceleration section is only one step. This step is removed and if ther ist a cruisingBeforeAcceleration section it will be combined with the new cruising section.
            energySavingStartId=get(csOriginal.behaviorSections, "cruisingBeforeAcceleration", get(csOriginal.behaviorSections, "acceleration", BehaviorSection())).dataPoints[1]
        end

        # TODO: should v_reach be reduced or is it enough to pop the data points?
            #    characteristicSection.v_reach=drivingCourse[end].v      # setting v_reach to the last data points velocity which is the highest reachable value in this characteristic section

        # copy the drivingCourse till the beginning of energy saving
        drivingCourseModified=Vector{DataPoint}()
        for i in 1:energySavingStartId
            push!(drivingCourseModified, DataPoint(drivingCourse[i]))  # List of data points till the start of energy saving
        end

        #s_braking=max(0.0, ceil((csModified.v_exit^2-csModified.v_reach^2)/2/train.a_braking, digits=approximationLevel))       # ceil is used to be sure that the train stops at s_end in spite of rounding errors
        s_braking=max(0.0, ceil((csModified.v_exit^2-drivingCourseModified[end].v^2)/2/train.a_braking, digits=approximationLevel))       # ceil is used to be sure that the train stops at s_end in spite of rounding errors
        s_cruising=csModified.s_end-drivingCourseModified[end].s-s_braking

        if s_cruising >0.001
            (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
        end #if

        # s_brakingAfterCruising=ceil((csModified.v_exit^2-drivingCourseModified[end].v^2)/2/train.a_braking, digits=10) #  TODO: check if s_braking and s_brakingAfterCruising are really always the same
        if drivingCourseModified[end].v>csModified.v_exit
            #(csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, train, allCSs)
            (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings, train, allCSs)
        elseif drivingCourseModified[end].s<csModified.s_end
            if (csModified.s_end-drivingCourseModified[end].s)>0.001
                # if (csModified.s_end-drivingCourseModified[end].s)>10^(-approximationLevel)
            #    println("INFO: The end of new CS",csModified.id," is not reached while saving energy with lowering v_reach.")
            #    println("      Therefore the calculation of this method can not continue for this CS.")
                return (CharacteristicSection(), [], false)
            end
            println("WARNING: The end of new CS",csModified.id," is not reached while saving energy with lowering v_reach.")
            println("         Therefore   s=",drivingCourseModified[end].s," will be set s_end=",csModified.s_end," because the difference is only ",csModified.s_end-drivingCourseModified[end].s," m.")
            println("          v=",drivingCourseModified[end].v," m/s   v_exit=",csOriginal.v_exit ," m/s")

            drivingCourseModified[end].s=csModified.s_end       # rounding up to s_end
        end #if

        if t_recoveryAvailable>=(csModified.t_total-csOriginal.t_total)
            return (csModified, drivingCourseModified, true)
        else # time loss is to high. so there is no energy saving modification for this CS with the available recovery time
            # 09/06 old: else # time loss is to high and the CS has to be calculated again with larger acceleration section (so with a smaller reduction of the acceleration section)
            # 09/06 old: accelerationReduction=min(accelerationReduction/10, csModified.v_reach-csModified.v_entry, csModified.v_reach-csModified.v_exit)
                # TODO: just return false or take smaller steps?

            return (CharacteristicSection(), [], false)
        end

    # 09/06 old:   end #while
    #
    # 09/06 old:   # there is no energy saving modification for this CS with the available recovery time
    # 09/06 old:   return (CharacteristicSection(), [], false)

    else
        # there is no energy saving modification for this CS because v_reach can not be lowered below v_entry or v_exit or because there is no acceleration section that can be transformed into a cruising section
        return (CharacteristicSection(), [], false)
    end #if haskey
end # function decreaseMaximumVelocity

# combination of method 1 and method 2
function combineEnergySavingMethods(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
 #    if haskey(csOriginal.behaviorSections, "acceleration") && (haskey(csOriginal.behaviorSections, "braking") || haskey(csOriginal.behaviorSections, "coasting")) && csOriginal.v_reach>csOriginal.v_entry && csOriginal.v_reach>csOriginal.v_exit
    if haskey(csOriginal.behaviorSections, "acceleration") && (haskey(csOriginal.behaviorSections, "braking") || haskey(csOriginal.behaviorSections, "coasting")) && drivingCourse[get(csOriginal.behaviorSections, "acceleration", BehaviorSection()).dataPoints[end]].v > max(csOriginal.v_entry, csOriginal.v_exit)
        csCombined=CharacteristicSection(csOriginal)
        drivingCourseCombined=Vector{DataPoint}()
        for i in 1:length(drivingCourse)
            # TODO: tried to insert copy on 15.07.2021 push!(drivingCourseCombined, copy(drivingCourse[i]))
            push!(drivingCourseCombined, DataPoint(drivingCourse[i]))
        end
        ΔE=0.0      # saved energy (in Ws)
        Δt=0.0      # time loss (in s)
        #while (Δt<t_recoveryAvailable && ΔE<=0.0)
        while (Δt<t_recoveryAvailable && (Δt<=0.0 || ΔE<=0.0)) #ΔE<=0.0)
            (csCombined, drivingCourseCombined, newMaximumVelocity)=decreaseMaximumVelocity(csCombined, drivingCourseCombined, settings, train, allCSs, t_recoveryAvailable)
            if newMaximumVelocity
                ΔE=csOriginal.E_total-csCombined.E_total      # saved energy (in Ws)
                Δt=csCombined.t_total-csOriginal.t_total      # time loss (in s)

                while (haskey(csOriginal.behaviorSections, "cruising") && (Δt<=0.0 || ΔE<=0.0)) #Δt<=0.0) #&& ΔE<=0.0) # && Δt<=0.0)
                    (csCombined, drivingCourseCombined, newCoasting)=increaseCoastingSection(csCombined, drivingCourseCombined, settings, train, allCSs, t_recoveryAvailable-Δt)
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

end #module EnergySaving
