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
            if train.m_train<=300000      # unit is kg
                c_t=0.03
            elseif train.m_train<=500000  # unit is kg
                c_t=0.04
            elseif train.m_train<=700000  # unit is kg
                c_t=0.04
            else # train.m_train>700000   # unit is kg
                c_t=0.05
            end # if train.m_train
        elseif train.v_limit<=160/3.6     # unit is m/s
            if train.m_train<=300000      # unit is kg
                c_t=0.03
            elseif train.m_train<=500000  # unit is kg
                c_t=0.04
            else # train.m_train>500000   # unit is kg
                c_t=0.0
            end # if train.m_train
        elseif train.v_limit<=200/3.6     # unit is m/s
            if train.m_train<=300000      # unit is kg
                c_t=0.04
            elseif train.m_train<=500000  # unit is kg
                c_t=0.05
            else # train.m_train>500000   # unit is kg
                c_t=0.06
            end # if train.m_train
        else # train.v_limit>200/3.6      # unit is m/s
            if train.m_train<=300000      # unit is kg
                c_t=0.05
            elseif train.m_train<=500000  # unit is kg
                c_t=0.06
            else # train.m_train>500000   # unit is kg
                c_t=0.07
            end # if train.m_train
        end # if train.v_limit

        c_tMin=s_MS/t_MS*0.0012
        c_t=max(c_t, c_tMin)

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    end # if train.trainType
end #function calculateRecoveryTime

function increaseCoastingSection(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if (haskey(csOriginal.behaviorSections, "cruising") || haskey(csOriginal.behaviorSections, "diminishing")) && haskey(csOriginal.behaviorSections, "braking")
        # check if cruising or diminishing should be reduced for coasting
        if haskey(csOriginal.behaviorSections, "cruising") && haskey(csOriginal.behaviorSections, "diminishing")
            if get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[1] > get(csOriginal.behaviorSections, "diminishing", BehaviorSection()).dataPoints[1]
                reduceCruising=true
                reduceDiminishing=false
            else
                reduceDiminishing=true
                reduceCruising=false
            end
        elseif haskey(csOriginal.behaviorSections, "cruising")
            reduceCruising=true
            reduceDiminishing=false
        elseif haskey(csOriginal.behaviorSections, "diminishing")
            reduceDiminishing=true
            reduceCruising=false
        end



        if reduceCruising
            cruisingReduction=settings.stepSize
            while cruisingReduction>=settings.stepSize/10^approximationLevel
            #while cruisingReduction>=settings.stepSize/100
                while cruisingReduction>=settings.stepSize/10^approximationLevel # will be done once and then depending on approximationLevel repeated with smaller cruisingReduction unless !(drivingCourseModified[end].v<=csModified.v_exit && drivingCourseModified[end].s<csModified.s_exit) -> see below at the end of the while loop

                    # create a copy for the characteristic sections drivingCourse
                    energySavingStartId=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[1]
                    if energySavingStartId==0
                        error("ERROR at creating a new driving course for energy saving with coasting !")
                    end

                    # copy the driving course till the beginning of energy saving
                    drivingCourseModified=Vector{DataPoint}()
                    for i in 1:energySavingStartId
                        push!(drivingCourseModified, DataPoint(drivingCourse[i]))  # List of data points till the start of energy saving
                        # TODO: tried to insert copy on 15.07.2021 push!(drivingCourseModified, copy(drivingCourse[i]))    # List of data points till the start of energy saving
                    end

                    # calculating the new length of the cruising section
                    if settings.stepVariable=="s in m"                                                           # distance step method
                        s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).length-cruisingReduction
                    elseif settings.stepVariable=="t in s"                                                       # time step method
                        # 09/20 old: doesn't work for non constant cruising
                            # t_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).t-cruisingReduction
                            # s_cruising=t_cruising*drivingCourseModified[end].v
                        wayReduction=drivingCourse(get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[end]).v*cruisingReduction
                        s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).length-wayReduction

                    elseif settings.stepVariable=="v in m/s"                                                       # velocity step method
                        s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).length-cruisingReduction*10 # TODO: or better: *100 ?
                    end #if
                    s_cruising=max(0.0, s_cruising)

                    # copy csOriginal to csModified
                    csModified=CharacteristicSection(csOriginal.id, csOriginal.length, csOriginal.s_entry, csOriginal.s_exit, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_target, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())
                    if haskey(csOriginal.behaviorSections, "breakFree")
                        breakFreeSection=BehaviorSection(get(csOriginal.behaviorSections, "breakFree", BehaviorSection()))
                        merge!(csModified.behaviorSections, Dict("breakFree"=>breakFreeSection))
                        csModified.E=csModified.E+get(csModified.behaviorSections, "breakFree", BehaviorSection()).E
                        csModified.t=csModified.t+get(csModified.behaviorSections, "breakFree", BehaviorSection()).t
                    end
                    if haskey(csOriginal.behaviorSections, "clearing")            # this section is needed before acceleration if the train wants to accelerate to a speed higher than the limit in a previous CS where parts of the train are still located
                        clearingSection=BehaviorSection(get(csOriginal.behaviorSections, "clearing", BehaviorSection()))
                        merge!(csModified.behaviorSections, Dict("clearing"=>clearingSection))
                        csModified.E=csModified.E+get(csModified.behaviorSections, "clearing", BehaviorSection()).E
                        csModified.t=csModified.t+get(csModified.behaviorSections, "clearing", BehaviorSection()).t
                    end
                    if haskey(csOriginal.behaviorSections, "acceleration")
                        accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))
                        merge!(csModified.behaviorSections, Dict("acceleration"=>accelerationSection))
                        csModified.E=csModified.E+get(csModified.behaviorSections, "acceleration", BehaviorSection()).E
                        csModified.t=csModified.t+get(csModified.behaviorSections, "acceleration", BehaviorSection()).t
                    end

                    if haskey(csOriginal.behaviorSections, "diminishing")
                        diminishingSection=BehaviorSection(get(csOriginal.behaviorSections, "diminishing", BehaviorSection()))
                        merge!(csModified.behaviorSections, Dict("diminishing"=>diminishingSection))
                        csModified.E=csModified.E+get(csModified.behaviorSections, "diminishing", BehaviorSection()).E
                        csModified.t=csModified.t+get(csModified.behaviorSections, "diminishing", BehaviorSection()).t
                    end


                    # calculate the new and now shorter cruising section
                    if s_cruising>0.0
                        (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
                    end

                    # calculate the coasting phase until the point the train needs to brake
                    (csModified, drivingCourseModified)=addCoastingPhaseUntilBraking!(csModified, drivingCourseModified, settings, train, allCSs)

                    if drivingCourseModified[end].v < csModified.v_exit || drivingCourseModified[end].s > csModified.s_exit
                        # the train reaches v_exit before reaching s_exit. The cruising and coasting sections have to be calculated again with a larger cruising section (so with a smaller reduction of the cruising section)
                        cruisingReduction=cruisingReduction/10
                    else
                        break
                    end
                end # while cruisingReduction

                # calculate the moving phase between coasting and the end of the CS
                if drivingCourseModified[end].v > csModified.v_exit
                    #(csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, train, allCSs)
                    (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings, train, allCSs)
                elseif drivingCourseModified[end].v == csModified.v_exit && drivingCourseModified[end].s < csModified.s_exit
                    # v_exit is already reached. Now cruise till the end of the CS
                    s_cruisingAfterCoasting=csModified.s_exit-drivingCourseModified[end].s
                    (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruisingAfterCoasting, settings, train, allCSs, "cruisingAfterCoasting")
                end

                if t_recoveryAvailable < csModified.t-csOriginal.t || drivingCourseModified[end].v != csModified.v_exit || drivingCourseModified[end].s != csModified.s_exit # time loss is to high and the CS has to be calculated again with larger cruising section (so with a smaller reduction of the cruising section) or v_exit or s_exit are not reached excatly
                    cruisingReduction=cruisingReduction/10
                else
                    return (csModified, drivingCourseModified, true)
                end
            end #while


        elseif reduceDiminishing
            # TODO: At the moment diminishing is reduced like the acceleration in decreaseMaximumVelocity. To reduce code, the methods for reducing cruising phase an reducing the diminishing pahse can be combined in some parts.

            # copy csOriginal to csModified
            csModified=CharacteristicSection(csOriginal.id, csOriginal.length, csOriginal.s_entry, csOriginal.s_exit, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_target, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())

            if haskey(csOriginal.behaviorSections, "breakFree")
                breakFreeSection=BehaviorSection(get(csOriginal.behaviorSections, "breakFree", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("breakFree"=>breakFreeSection))
                csModified.E=csModified.E+get(csModified.behaviorSections, "breakFree", BehaviorSection()).E
                csModified.t=csModified.t+get(csModified.behaviorSections, "breakFree", BehaviorSection()).t
            end

            if haskey(csOriginal.behaviorSections, "clearing")            # this section is needed before acceleration if the train wants to accelerate to a speed higher than the limit in a previous CS where parts of the train are still located
                clearingSection=BehaviorSection(get(csOriginal.behaviorSections, "clearing", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("clearing"=>clearingSection))
                csModified.E=csModified.E+get(csModified.behaviorSections, "clearing", BehaviorSection()).E
                csModified.t=csModified.t+get(csModified.behaviorSections, "clearing", BehaviorSection()).t
            end
            if haskey(csOriginal.behaviorSections, "acceleration")
                accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("acceleration"=>accelerationSection))
                csModified.E=csModified.E+get(csModified.behaviorSections, "acceleration", BehaviorSection()).E
                csModified.t=csModified.t+get(csModified.behaviorSections, "acceleration", BehaviorSection()).t
            end

            if haskey(csOriginal.behaviorSections, "cruising")
                cruisingSection=BehaviorSection(get(csOriginal.behaviorSections, "cruising", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("cruising"=>cruisingSection))
                csModified.E=csModified.E+get(csModified.behaviorSections, "cruising", BehaviorSection()).E
                csModified.t=csModified.t+get(csModified.behaviorSections, "cruising", BehaviorSection()).t
            end

            diminishingSection=BehaviorSection(get(csOriginal.behaviorSections, "diminishing", BehaviorSection()))
            if length(diminishingSection.dataPoints) > 2
                # remove the last diminishing waypoint
                pop!(diminishingSection.dataPoints)

                diminishingSection.v_exit=drivingCourse[diminishingSection.dataPoints[end]].v                               # exit speed (in m/s)
                diminishingSection.s_exit=drivingCourse[diminishingSection.dataPoints[end]].s                                # last position (in m)
                diminishingSection.length=diminishingSection.s_exit-diminishingSection.s_entry           # total length  (in m)
                diminishingSection.t=drivingCourse[diminishingSection.dataPoints[end]].t-drivingCourse[diminishingSection.dataPoints[1]].t       # total running time (in s)
                diminishingSection.E=drivingCourse[diminishingSection.dataPoints[end]].E-drivingCourse[diminishingSection.dataPoints[1]].E       # total energy consumption (in Ws)

                merge!(csModified.behaviorSections, Dict("diminishing"=>diminishingSection))
                csModified.E=csModified.E+get(csModified.behaviorSections, "diminishing", BehaviorSection()).E
                csModified.t=csModified.t+get(csModified.behaviorSections, "diminishing", BehaviorSection()).t

                energySavingStartId=diminishingSection.dataPoints[end]
            else
                # The diminishing section is only one step. This step is removed and if there is a clearing section it will be combined with the new cruising section.
                energySavingStartId=get(csOriginal.behaviorSections, "clearing", get(csOriginal.behaviorSections, "diminishing", BehaviorSection())).dataPoints[1]
            end

            # copy the driving course till the beginning of energy saving
            drivingCourseModified=Vector{DataPoint}()
            for i in 1:energySavingStartId
                push!(drivingCourseModified, DataPoint(drivingCourse[i]))  # List of data points till the start of energy saving
            end

            # calculate the coasting phase until the point the train needs to brake
            (csModified, drivingCourseModified)=addCoastingPhaseUntilBraking!(csModified, drivingCourseModified, settings, train, allCSs)

            # calculate the moving phase between coasting and the end of the CS
            if drivingCourseModified[end].v > csModified.v_exit
                (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings, train, allCSs)
            elseif drivingCourseModified[end].v == csModified.v_exit && drivingCourseModified[end].s < csModified.s_exit
                # v_exit is already reached. Now cruise till the end of the CS
                s_cruisingAfterCoasting=csModified.s_exit-drivingCourseModified[end].s
                (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruisingAfterCoasting, settings, train, allCSs, "cruisingAfterCoasting")
            end


            if t_recoveryAvailable>=(csModified.t-csOriginal.t)
                return (csModified, drivingCourseModified, true)
            else # time loss is to high. so there is no energy saving modification for this CS with the available recovery time
                # TODO: just return false or take smaller steps?

                return (CharacteristicSection(), [], false)
            end
        end

        # there is no energy saving modification for this CS with the available recovery time
        return (CharacteristicSection(), [], false)
    else
        # there is no energy saving modification for this CS because a cruising section AND a braking section are needed to be transformed into a coasting section
        return (CharacteristicSection(), [], false)
    end
end # function increaseCoastingSection

#= 12/03 old without diminishing: function increaseCoastingSection(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "cruising") && haskey(csOriginal.behaviorSections, "braking")
        cruisingReduction=settings.stepSize
        while cruisingReduction>=settings.stepSize/10^approximationLevel
        #while cruisingReduction>=settings.stepSize/100
            while cruisingReduction>=settings.stepSize/10^approximationLevel # will be done once and then depending on approximationLevel repeated with smaller cruisingReduction unless !(drivingCourseModified[end].v<=csModified.v_exit && drivingCourseModified[end].s<csModified.s_exit) -> see below at the end of the while loop

                # create a copy for the characteristic sections drivingCourse
                energySavingStartId=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[1]
                if energySavingStartId==0
                    error("ERROR at creating a new driving course for energy saving with coasting !")
                end

                drivingCourseModified=[DataPoint(drivingCourse[1])]
                    # TODO: tried to insert copy on 15.07.2021 drivingCourseModified=[copy(drivingCourse[1])]
                for i in 2:energySavingStartId
                    push!(drivingCourseModified, DataPoint(drivingCourse[i]))    # List of data points till the start of energy saving
                    # TODO: tried to insert copy on 15.07.2021 push!(drivingCourseModified, copy(drivingCourse[i]))    # List of data points till the start of energy saving

                end

                # calculating the new length of the cruising section
                if settings.stepVariable=="s in m"                                                           # distance step method
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).length-cruisingReduction
                elseif settings.stepVariable=="t in s"                                                       # time step method
                    # 09/20 old: doesn't work for non constant cruising
                        # t_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).t-cruisingReduction
                        # s_cruising=t_cruising*drivingCourseModified[end].v
                    wayReduction=drivingCourse(get(csOriginal.behaviorSections, "cruising", BehaviorSection()).dataPoints[end]).v*cruisingReduction
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).length-wayReduction

                elseif settings.stepVariable=="v in m/s"                                                       # velocity step method
                    s_cruising=get(csOriginal.behaviorSections, "cruising", BehaviorSection()).length-cruisingReduction*10 # TODO: or better: *100 ?
                end #if
                s_cruising=max(0.0, s_cruising)

                # copy csOriginal to csModified
                csModified=CharacteristicSection(csOriginal.id, csOriginal.length, csOriginal.s_entry, csOriginal.s_exit, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_target, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())
                if haskey(csOriginal.behaviorSections, "breakFree")
                    breakFreeSection=BehaviorSection(get(csOriginal.behaviorSections, "breakFree", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("breakFree"=>breakFreeSection))
                    csModified.E=csModified.E+get(csModified.behaviorSections, "breakFree", BehaviorSection()).E
                    csModified.t=csModified.t+get(csModified.behaviorSections, "breakFree", BehaviorSection()).t
                end
                if haskey(csOriginal.behaviorSections, "clearing")            # this section is needed before acceleration if the train wants to accelerate to a speed higher than the limit in a previous CS where parts of the train are still located
                    clearingSection=BehaviorSection(get(csOriginal.behaviorSections, "clearing", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("clearing"=>clearingSection))
                    csModified.E=csModified.E+get(csModified.behaviorSections, "clearing", BehaviorSection()).E
                    csModified.t=csModified.t+get(csModified.behaviorSections, "clearing", BehaviorSection()).t
                end
                if haskey(csOriginal.behaviorSections, "acceleration")
                    accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))
                    merge!(csModified.behaviorSections, Dict("acceleration"=>accelerationSection))
                    csModified.E=csModified.E+get(csModified.behaviorSections, "acceleration", BehaviorSection()).E
                    csModified.t=csModified.t+get(csModified.behaviorSections, "acceleration", BehaviorSection()).t
                end

                # calculate the new and now shorter cruising section
                if s_cruising>0.0
                    (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
                end

                (csModified, drivingCourseModified)=addCoastingPhaseUntilBraking!(csModified, drivingCourseModified, settings, train, allCSs)

                if drivingCourseModified[end].v < csModified.v_exit || drivingCourseModified[end].s > csModified.s_exit
                    # the train reaches v_exit before reaching s_exit. The cruising and coasting sections have to be calculated again with a larger cruising section (so with a smaller reduction of the cruising section)
                    cruisingReduction=cruisingReduction/10
                else
                    break
                end
            end # while cruisingReduction

            if drivingCourseModified[end].v > csModified.v_exit
                #(csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, train, allCSs)
                (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings, train, allCSs)
            elseif drivingCourseModified[end].v == csModified.v_exit && drivingCourseModified[end].s < csModified.s_exit
                # v_exit is already reached. Now cruise till the end of the CS
                s_cruisingAfterCoasting=csModified.s_exit-drivingCourseModified[end].s
                (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruisingAfterCoasting, settings, train, allCSs, "cruisingAfterCoasting")
            end

            if t_recoveryAvailable < csModified.t-csOriginal.t || drivingCourseModified[end].v != csModified.v_exit || drivingCourseModified[end].s != csModified.s_exit # time loss is to high and the CS has to be calculated again with larger cruising section (so with a smaller reduction of the cruising section) or v_exit or s_exit are not reached excatly
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

=#

# method 2 with shortening the acceleration by stepsize
function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
 #function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    if haskey(csOriginal.behaviorSections, "acceleration") && csOriginal.v_target > csOriginal.v_entry && csOriginal.v_target > csOriginal.v_exit
        accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))
        if drivingCourse[accelerationSection.dataPoints[end]-1].v < csOriginal.v_exit
            return (CharacteristicSection(), [], false)
            # TODO: or calculate a new acceleration phase with v_exit as v_target? it will be very short, shorter than the step size.
        end

        # copy csOriginal to csModified
        csModified=CharacteristicSection(csOriginal.id, csOriginal.length, csOriginal.s_entry, csOriginal.s_exit, 0.0, 0.0, csOriginal.v_limit, csOriginal.v_target, csOriginal.v_entry, csOriginal.v_exit, csOriginal.f_Rp, Dict{String, BehaviorSection}())

        if haskey(csOriginal.behaviorSections, "breakFree")
            breakFreeSection=BehaviorSection(get(csOriginal.behaviorSections, "breakFree", BehaviorSection()))
            merge!(csModified.behaviorSections, Dict("breakFree"=>breakFreeSection))
            csModified.E=csModified.E+get(csModified.behaviorSections, "breakFree", BehaviorSection()).E
            csModified.t=csModified.t+get(csModified.behaviorSections, "breakFree", BehaviorSection()).t
        end

        #accelerationSection=BehaviorSection(get(csOriginal.behaviorSections, "acceleration", BehaviorSection()))

        if length(accelerationSection.dataPoints) > 2
            if haskey(csOriginal.behaviorSections, "clearing")
                clearingSection=BehaviorSection(get(csOriginal.behaviorSections, "clearing", BehaviorSection()))
                merge!(csModified.behaviorSections, Dict("clearing"=>clearingSection))
                csModified.E=csModified.E+get(csModified.behaviorSections, "clearing", BehaviorSection()).E
                csModified.t=csModified.t+get(csModified.behaviorSections, "clearing", BehaviorSection()).t
            end

            # remove the last acceleration waypoint
            pop!(accelerationSection.dataPoints)

            accelerationSection.v_exit=drivingCourse[accelerationSection.dataPoints[end]].v                               # exit speed (in m/s)
            accelerationSection.s_exit=drivingCourse[accelerationSection.dataPoints[end]].s                                # last position (in m)
            accelerationSection.length=accelerationSection.s_exit-accelerationSection.s_entry           # total length  (in m)
            accelerationSection.t=drivingCourse[accelerationSection.dataPoints[end]].t-drivingCourse[accelerationSection.dataPoints[1]].t       # total running time (in s)
            accelerationSection.E=drivingCourse[accelerationSection.dataPoints[end]].E-drivingCourse[accelerationSection.dataPoints[1]].E       # total energy consumption (in Ws)

            merge!(csModified.behaviorSections, Dict("acceleration"=>accelerationSection))
            csModified.E=csModified.E+get(csModified.behaviorSections, "acceleration", BehaviorSection()).E
            csModified.t=csModified.t+get(csModified.behaviorSections, "acceleration", BehaviorSection()).t

            energySavingStartId=accelerationSection.dataPoints[end]
        else
            # The acceleration section is only one step. This step is removed and if there is a clearing section it will be combined with the new cruising section.
            energySavingStartId=get(csOriginal.behaviorSections, "clearing", get(csOriginal.behaviorSections, "acceleration", BehaviorSection())).dataPoints[1]
        end

        # TODO: should v_target be reduced or is it enough to pop the data points?
            #    characteristicSection.v_target=drivingCourse[end].v      # setting v_target to the last data point's velocity which is the highest reachable value in this characteristic section

        # copy the drivingCourse till the beginning of energy saving
        drivingCourseModified=Vector{DataPoint}()
        for i in 1:energySavingStartId
            push!(drivingCourseModified, DataPoint(drivingCourse[i]))  # List of data points till the start of energy saving
        end

        #s_braking=max(0.0, ceil((csModified.v_exit^2-csModified.v_target^2)/2/train.a_braking, digits=approximationLevel))       # ceil is used to be sure that the train stops at s_exit in spite of rounding errors
        s_braking=max(0.0, ceil((csModified.v_exit^2-drivingCourseModified[end].v^2)/2/train.a_braking, digits=approximationLevel))       # ceil is used to be sure that the train stops at s_exit in spite of rounding errors
        s_cruising=csModified.s_exit-drivingCourseModified[end].s-s_braking

        if s_cruising >0.001
            (csModified, drivingCourseModified)=addCruisingPhase!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
        end #if

        # s_brakingAfterCruising=ceil((csModified.v_exit^2-drivingCourseModified[end].v^2)/2/train.a_braking, digits=10) #  TODO: check if s_braking and s_brakingAfterCruising are really always the same
        if drivingCourseModified[end].v>csModified.v_exit
            #(csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings.massModel, train, allCSs)
            (csModified, drivingCourseModified)=addBrakingPhase!(csModified, drivingCourseModified, settings, train, allCSs)
        elseif drivingCourseModified[end].s<csModified.s_exit
            if (csModified.s_exit-drivingCourseModified[end].s)>0.001
                # if (csModified.s_exit-drivingCourseModified[end].s)>10^(-approximationLevel)
            #    println("INFO: The end of new CS",csModified.id," is not reached while saving energy with lowering v_target.")
            #    println("      Therefore the calculation of this method can not continue for this CS.")
                return (CharacteristicSection(), [], false)
            end
            println("WARNING: The end of new CS",csModified.id," is not reached while saving energy with lowering v_target.")
            println("         Therefore   s=",drivingCourseModified[end].s," will be set s_exit=",csModified.s_exit," because the difference is only ",csModified.s_exit-drivingCourseModified[end].s," m.")
            println("          v=",drivingCourseModified[end].v," m/s   v_exit=",csOriginal.v_exit ," m/s")

            drivingCourseModified[end].s=csModified.s_exit       # rounding up to s_exit
        end #if

        if t_recoveryAvailable>=(csModified.t-csOriginal.t)
            return (csModified, drivingCourseModified, true)
        else # time loss is to high. so there is no energy saving modification for this CS with the available recovery time
            # 09/06 old: else # time loss is to high and the CS has to be calculated again with larger acceleration section (so with a smaller reduction of the acceleration section)
            # 09/06 old: accelerationReduction=min(accelerationReduction/10, csModified.v_target-csModified.v_entry, csModified.v_target-csModified.v_exit)
                # TODO: just return false or take smaller steps?

            return (CharacteristicSection(), [], false)
        end

    # 09/06 old:   end #while
    #
    # 09/06 old:   # there is no energy saving modification for this CS with the available recovery time
    # 09/06 old:   return (CharacteristicSection(), [], false)

    else
        # there is no energy saving modification for this CS because v_target can not be lowered below v_entry or v_exit or because there is no acceleration section that can be transformed into a cruising section
        return (CharacteristicSection(), [], false)
    end #if haskey
end # function decreaseMaximumVelocity

# combination of method 1 and method 2
function combineEnergySavingMethods(csOriginal::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Settings, train::Train, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
 #    if haskey(csOriginal.behaviorSections, "acceleration") && (haskey(csOriginal.behaviorSections, "braking") || haskey(csOriginal.behaviorSections, "coasting")) && csOriginal.v_target>csOriginal.v_entry && csOriginal.v_target>csOriginal.v_exit
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
                ΔE=csOriginal.E-csCombined.E      # saved energy (in Ws)
                Δt=csCombined.t-csOriginal.t      # time loss (in s)

                while (haskey(csOriginal.behaviorSections, "cruising") && (Δt<=0.0 || ΔE<=0.0)) #Δt<=0.0) #&& ΔE<=0.0) # && Δt<=0.0)
                    (csCombined, drivingCourseCombined, newCoasting)=increaseCoastingSection(csCombined, drivingCourseCombined, settings, train, allCSs, t_recoveryAvailable-Δt)
                    if newCoasting
                        ΔE=csOriginal.E-csCombined.E      # saved energy (in Ws)
                        Δt=csCombined.t-csOriginal.t      # time loss (in s)
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
        # there is no energy saving modification for this CS because v_target can not be lowered below v_entry or v_exit or because there is no acceleration section and braking section or coasting section that can be transformed into a cruising section or coasting section
        return (CharacteristicSection(), [], false)
    end #if
end #function combineEnergySavingMethods

end #module EnergySaving
