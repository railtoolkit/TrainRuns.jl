module RailwayDrivingDynamicsOperationModes

using ..types

include("./RailwayDrivingDynamicsMovingPhases.jl")
include("./RailwayDrivingDynamicsEnergySaving.jl")
using .RailwayDrivingDynamicsMovingPhases
using .RailwayDrivingDynamicsEnergySaving

export simulateMinimumRunningTime!, simulateMinimumEnergyConsumption

function simulateMinimumRunningTime!(movingSection::MovingSection, settings::Settings, vehicle::Vehicle)
    # this function simulates a train run focussing on using the minimum possible running time
    startingPoint=Waypoint()
    startingPoint.i=1
    drivingCourse=[startingPoint]    # List of waypoints
    drivingCourse[end].s=movingSection.characteristicSections[1].s_start

    for csId in 1:length(movingSection.characteristicSections)
        # check if the CS has a cruising section
        s_starting=get(movingSection.characteristicSections[csId].behaviorSections, "starting", BehaviorSection()).s_total
        s_cruisingBeforeAcceleration=get(movingSection.characteristicSections[csId].behaviorSections, "cruisingBeforeAcceleration", BehaviorSection()).s_total
        s_acceleration=get(movingSection.characteristicSections[csId].behaviorSections, "acceleration", BehaviorSection()).s_total
        s_braking=max(0.0, ceil((movingSection.characteristicSections[csId].v_exit^2-movingSection.characteristicSections[csId].v_reach^2)/2/vehicle.a_braking, digits=10))   # ceil is used to be sure that the vehicle stops at s_end in spite of rounding errors

        s_cruising=movingSection.characteristicSections[csId].s_total-s_starting-s_cruisingBeforeAcceleration-s_acceleration-s_braking

        # reset the moving section (MS), delete characteristic sections (CS) that were used during the preperation for setting v_entry, v_reach and v_exit
        delete!(movingSection.characteristicSections[csId].behaviorSections, "starting")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "cruisingBeforeAcceleration")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "acceleration")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "cruising")
        movingSection.characteristicSections[csId].E_total=0.0
        movingSection.characteristicSections[csId].t_total=0.0

        if s_cruisingBeforeAcceleration == movingSection.characteristicSections[csId].s_total
            (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruisingBeforeAcceleration, settings, vehicle, movingSection.characteristicSections, "cruisingBeforeAcceleration")

        elseif s_cruising == movingSection.characteristicSections[csId].s_total
            (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruising, settings, vehicle, movingSection.characteristicSections, "cruising")

        elseif s_cruising >0.01 # if the cruising section is longer than 1 cm (because of rounding issues not >0.0)
            if drivingCourse[end].v<movingSection.characteristicSections[csId].v_reach
                (movingSection.characteristicSections[csId], drivingCourse)=addAccelerationPhase!(movingSection.characteristicSections[csId], drivingCourse, settings, vehicle, movingSection.characteristicSections)
            end #if

            if (movingSection.characteristicSections[csId].s_end-drivingCourse[end].s-max(0.0, (movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/vehicle.a_braking))<-0.001   # ceil is used to be sure that the vehicle stops at s_end in spite of rounding errors
                println("ERROR: After accelerating in CS ",csId," the braking distance is too short!")
                println("     before acceleration in CS",csId, "  with s=",drivingCourse[end].s,"  s_braking=",((movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/vehicle.a_braking),"   s_end=",movingSection.characteristicSections[csId].s_end)
                println("                             and v=",drivingCourse[end].v,"   v_reach=",movingSection.characteristicSections[csId].v_reach,"  v_exit=",movingSection.characteristicSections[csId].v_exit)
            end

            s_braking=max(0.0, ceil((movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/vehicle.a_braking, digits=10))   # ceil is used to be sure that the vehicle stops at s_end in spite of rounding errors
            s_cruising=movingSection.characteristicSections[csId].s_end-drivingCourse[end].s-s_braking

            if s_cruising>0
                (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruising, settings, vehicle, movingSection.characteristicSections, "cruising")
            end
        else
            if movingSection.characteristicSections[csId].v_entry<movingSection.characteristicSections[csId].v_reach
                (movingSection.characteristicSections[csId], drivingCourse)=addAccelerationPhaseWithIntersection!(movingSection.characteristicSections[csId], drivingCourse, settings, vehicle, movingSection.characteristicSections)
            end #if
        end #if


        s_braking=max(0.0, ceil((movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/vehicle.a_braking, digits=10))     # ceil is used to be sure that the vehicle stops at s_end in spite of rounding errors

        if drivingCourse[end].v>movingSection.characteristicSections[csId].v_exit
            (movingSection.characteristicSections[csId], drivingCourse)=addBrakingPhase!(movingSection.characteristicSections[csId], drivingCourse, settings.massModel, vehicle, movingSection.characteristicSections)
        end #if

        if drivingCourse[end].s < movingSection.characteristicSections[csId].s_end
            if haskey(movingSection.characteristicSections[csId].behaviorSections, "cruising")
                println("INFO: A second cruising section has been added to CS ", csId," from s=",drivingCourse[end].s,"  to s_end=",movingSection.characteristicSections[csId].s_end)
            end
            (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruising, settings, vehicle, movingSection.characteristicSections, "cruising")
        end
    end #for

    movingSection.t_total=drivingCourse[end].t            # total running time (in s)
    movingSection.E_total=drivingCourse[end].E            # total energy consumption (in Ws)

    return (movingSection, drivingCourse)
end #function simulateMinimumRunningTime


function simulateMinimumEnergyConsumption(movingSectionMinimumRunningTime::MovingSection, drivingCourseMinimumRunningTime::Vector{Waypoint}, settings::Settings, vehicle::Vehicle)
# this function simulates a train run focussing on using the minimum possible energy consumption
    # booleans for choosing which methods are used for saving energy
    doMethod1=true
    doMethod2=true
    doCombinationOfMethods=true

    #creating a new moving section for the minimum energy consumption
    movingSectionOriginal=MovingSection(movingSectionMinimumRunningTime)

    # creating a new driving course for the minimum energy consumption
    drivingCourseOriginal=Waypoint[]
    for i in 1:length(drivingCourseMinimumRunningTime)
        push!(drivingCourseOriginal, Waypoint(drivingCourseMinimumRunningTime[i]))    # List of waypoints till the start of energy saving
    end

    # calculation the recovery time
    movingSectionOriginal.t_recovery=calculateRecoveryTime(movingSectionOriginal.s_total, movingSectionOriginal.t_total, vehicle)
    movingSectionOriginal.t_recoveryAvailable=movingSectionOriginal.t_recovery
    # println("t_recovery=",movingSectionOriginal.t_recovery)

    # creating arrays for each method with all the available energy saving modifications
    energySavingModificationsWithCoasting=EnergySavingModification[]
    energySavingModificationsWithMaximumSpeed=EnergySavingModification[]
    energySavingModificationsWithCombination=EnergySavingModification[]

    for csId in 1:length(movingSectionOriginal.characteristicSections)
        # method 1: increase coasting
        if doMethod1==true
            (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=increaseCostingSection(movingSectionOriginal.characteristicSections[csId], drivingCourseOriginal, settings, vehicle, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
            if new
                energySavingModification=EnergySavingModification()
                energySavingModification.csId=csId                                                                                                      # identifier of the characteristic section
                energySavingModification.type="increasing coasting"                                                                                     # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
                energySavingModification.csModified=characteristicSectionModified                                                                       # the modified characteristic section
                energySavingModification.drivingCourseModified=drivingCourseModifiedUntilEndOfModifiedCS                                                # drivingCourse of the modified characteristic section
                energySavingModification.ΔE=movingSectionOriginal.characteristicSections[csId].E_total-energySavingModification.csModified.E_total      # saved energy (in Ws)
                energySavingModification.Δt=energySavingModification.csModified.t_total-movingSectionOriginal.characteristicSections[csId].t_total      # time loss (in s)
                if energySavingModification.Δt<=movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE>=0.0
                    energySavingModification.ratio=energySavingModification.ΔE/energySavingModification.Δt                                              # ratio of ΔE and Δt (in Ws/s)
                elseif energySavingModification.Δt==0.0 #
                    energySavingModification.ratio=energySavingModification.ΔE/0.000000001
                else # Δt is to high
                    energySavingModification.ratio=0.0
                end
                push!(energySavingModificationsWithCoasting, energySavingModification)
            else # there is no modification possible for the current CS
                push!(energySavingModificationsWithCoasting, EnergySavingModification())
            end
        end #if doMethod1

        # method 2: accelerate to a lower v_reach
        if doMethod2==true
            (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=decreaseMaximumVelocity(movingSectionOriginal.characteristicSections[csId], drivingCourseOriginal, settings, vehicle, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
            if new
                energySavingModification=EnergySavingModification()
                energySavingModification.csId=csId                                                                                                      # identifier of the characteristic section
                energySavingModification.type="decreasing maximum velocity"                                                                             # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
                energySavingModification.csModified=characteristicSectionModified                                                                       # the modified characteristic section
                energySavingModification.drivingCourseModified=drivingCourseModifiedUntilEndOfModifiedCS                                                # drivingCourse of the modified characteristic section
                energySavingModification.ΔE=movingSectionOriginal.characteristicSections[csId].E_total-energySavingModification.csModified.E_total      # saved energy (in Ws)
                energySavingModification.Δt=energySavingModification.csModified.t_total-movingSectionOriginal.characteristicSections[csId].t_total      # time loss (in s)
                if energySavingModification.Δt<=movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE>=0.0
                    energySavingModification.ratio=energySavingModification.ΔE/energySavingModification.Δt                                              # ratio of ΔE and Δt (in Ws/s)
                elseif energySavingModification.Δt==0.0
                    energySavingModification.ratio=energySavingModification.ΔE/0.000000001
                else # Δt is to high or ΔE < 0.0 Ws
                    energySavingModification.ratio=0.0
                end
                push!(energySavingModificationsWithMaximumSpeed, energySavingModification)
            else # there is no modification possible for the current CS
                push!(energySavingModificationsWithMaximumSpeed, EnergySavingModification())
            end
        end #if doMethod2


        if doCombinationOfMethods==true
            # calculating the combination of methods
            (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=combineEnergySavingMethods(movingSectionOriginal.characteristicSections[csId], drivingCourseOriginal, settings, vehicle, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
            if new
                energySavingModification=EnergySavingModification()
                energySavingModification.csId=csId                                                                                                  # identifier of the characteristic section
                energySavingModification.type="combination of energy saving methods"                                             # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
                energySavingModification.csModified=characteristicSectionModified                                                                   # the modified characteristic section
                energySavingModification.drivingCourseModified=drivingCourseModifiedUntilEndOfModifiedCS                                            # drivingCourse of the modified characteristic section
                energySavingModification.ΔE=movingSectionOriginal.characteristicSections[csId].E_total-energySavingModification.csModified.E_total  # saved energy (in Ws)
                energySavingModification.Δt=energySavingModification.csModified.t_total-movingSectionOriginal.characteristicSections[csId].t_total  # time loss (in s)
                if energySavingModification.Δt<=movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE>=0.0
                    energySavingModification.ratio=sign(energySavingModification.Δt)*energySavingModification.ΔE/energySavingModification.Δt        # ratio of ΔE and Δt (in Ws/s)
                elseif energySavingModification.Δt==0.0 #
                    energySavingModification.ratio=energySavingModification.ΔE/0.000000001
                else # Δt is to high or ΔE < 0.0 Ws
                    energySavingModification.ratio=0.0
                end
                push!(energySavingModificationsWithCombination, energySavingModification)
            else # there is no modification possible for the current CS
                push!(energySavingModificationsWithCombination, EnergySavingModification())
            end
        end #if
    end # for


    while (movingSectionOriginal.t_recoveryAvailable>0.0)
        # comparison of modifications
        ratioMax=0.0
        csIdMax=0
        typeMax="none"
        for modNr in 1:length(energySavingModificationsWithMaximumSpeed)
            if energySavingModificationsWithMaximumSpeed[modNr].ratio>ratioMax
                if energySavingModificationsWithMaximumSpeed[modNr].Δt<=movingSectionOriginal.t_recoveryAvailable
                    ratioMax=energySavingModificationsWithMaximumSpeed[modNr].ratio
                    csIdMax=energySavingModificationsWithMaximumSpeed[modNr].csId
                    typeMax=energySavingModificationsWithMaximumSpeed[modNr].type
                else # Δt is to high
                    energySavingModificationsWithMaximumSpeed[modNr].ratio=0.0
                end #if
            end #if
        end #for

        for modNr in 1:length(energySavingModificationsWithCoasting)
            if energySavingModificationsWithCoasting[modNr].ratio>ratioMax
                if energySavingModificationsWithCoasting[modNr].Δt<=movingSectionOriginal.t_recoveryAvailable
                    ratioMax=energySavingModificationsWithCoasting[modNr].ratio
                    csIdMax=energySavingModificationsWithCoasting[modNr].csId
                    typeMax=energySavingModificationsWithCoasting[modNr].type
                else # Δt is to high
                    energySavingModificationsWithCoasting[modNr].ratio=0.0
                end #if
            end #if
        end #for

        for modNr in 1:length(energySavingModificationsWithCombination)
            if energySavingModificationsWithCombination[modNr].ratio>ratioMax
                if energySavingModificationsWithCombination[modNr].Δt<=movingSectionOriginal.t_recoveryAvailable
                    ratioMax=energySavingModificationsWithCombination[modNr].ratio
                    csIdMax=energySavingModificationsWithCombination[modNr].csId
                    typeMax=energySavingModificationsWithCombination[modNr].type
                else # Δt is to high
                    energySavingModificationsWithCombination[modNr].ratio=0.0
                end #if
            end #if
        end #for


        # selection of most efficient modification and update the original characteristicSection, drivingCourse and movingSection
        # in case none of the modifications has a ratio>0 stop the calculation
        if typeMax=="none"
             break
        elseif typeMax=="increasing coasting"
        #    println("Energy saving modification number ",length(movingSectionOriginal.energySavingModifications)+1," (coasting) in CS ",csIdMax ," Δt=",energySavingModificationsWithCoasting[csIdMax].Δt," ΔE=", energySavingModificationsWithCoasting[csIdMax].ΔE," t_recoveryAvailable=", movingSectionOriginal.t_recoveryAvailable-energySavingModificationsWithCoasting[csIdMax].Δt)
            push!(movingSectionOriginal.energySavingModifications, energySavingModificationsWithCoasting[csIdMax])
        elseif typeMax=="decreasing maximum velocity"
        #    println("Energy saving modification number ",length(movingSectionOriginal.energySavingModifications)+1," (lowering v_reach) in CS ",csIdMax ," Δt=",energySavingModificationsWithCoasting[csIdMax].Δt," ΔE=", energySavingModificationsWithCoasting[csIdMax].ΔE," t_recoveryAvailable=", movingSectionOriginal.t_recoveryAvailable-energySavingModificationsWithCoasting[csIdMax].Δt)
            push!(movingSectionOriginal.energySavingModifications, energySavingModificationsWithMaximumSpeed[csIdMax])
        elseif typeMax=="combination of energy saving methods"
        #    println("Energy saving modification number ",length(movingSectionOriginal.energySavingModifications)+1," (combination) in CS ",csIdMax ," Δt=",energySavingModificationsWithCoasting[csIdMax].Δt," ΔE=", energySavingModificationsWithCoasting[csIdMax].ΔE," t_recoveryAvailable=", movingSectionOriginal.t_recoveryAvailable-energySavingModificationsWithCoasting[csIdMax].Δt)
            push!(movingSectionOriginal.energySavingModifications, energySavingModificationsWithCombination[csIdMax])
        end #if

        movingSectionOriginal.t_recoveryAvailable=movingSectionOriginal.t_recoveryAvailable-movingSectionOriginal.energySavingModifications[end].Δt

        lastIdOfModifiedCs=get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "braking", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "cruisingAfterCoasting", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "coasting", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "cruising", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "acceleration", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "cruisingBeforeAcceleration", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "starting", BehaviorSection()))))))).waypoints[end]

        drivingCourseNew=Vector{Waypoint}()
        for i in 1:length(movingSectionOriginal.energySavingModifications[end].drivingCourseModified)
            push!(drivingCourseNew, Waypoint(movingSectionOriginal.energySavingModifications[end].drivingCourseModified[i]))
        end

    #    # for testing if the modifications driving course and the original one have the same speed value at their transition point
    #    if lastIdOfModifiedCs+1<=drivingCourseOriginal[end].i
    #        if drivingCourseOriginal[lastIdOfModifiedCs].v-drivingCourseNew[end].v!=0.0
    #            println("INFO: Inconsistency while adding the CS",csIdMax," that has been modified for consuming less energy. The difference at its end is ",(drivingCourseOriginal[lastIdOfModifiedCs].v-drivingCourseNew[end].v)," m/s."
    #        end
    #    end # for testing

        #filling up the rest of the driving course
        drivingCourseNew[end].F_T=drivingCourseOriginal[lastIdOfModifiedCs].F_T
        drivingCourseNew[end].F_Rt=drivingCourseOriginal[lastIdOfModifiedCs].F_Rt
        drivingCourseNew[end].F_Rw=drivingCourseOriginal[lastIdOfModifiedCs].F_Rw
        drivingCourseNew[end].F_Runion=drivingCourseOriginal[lastIdOfModifiedCs].F_Runion
        drivingCourseNew[end].F_Rp=drivingCourseOriginal[lastIdOfModifiedCs].F_Rp
        drivingCourseNew[end].F_R=drivingCourseOriginal[lastIdOfModifiedCs].F_R
        drivingCourseNew[end].a=drivingCourseOriginal[lastIdOfModifiedCs].a

        endOfModificationId=drivingCourseNew[end].i                 # is needed for updating the other modified driving courses
        difference=drivingCourseNew[end].i-lastIdOfModifiedCs

        i=lastIdOfModifiedCs+1
        while (i<=length(drivingCourseOriginal))
            push!(drivingCourseNew, Waypoint(drivingCourseOriginal[i]))
            drivingCourseNew[end].i=length(drivingCourseNew)
            drivingCourseNew[end].t=drivingCourseNew[end-1].t+drivingCourseNew[end].Δt
            drivingCourseNew[end].E=drivingCourseNew[end-1].E+drivingCourseNew[end].ΔE
            drivingCourseNew[end].W_T=drivingCourseNew[end-1].W_T+drivingCourseNew[end].ΔW_T
            i=i+1
        end # while

        # updating all the waypoint references in the behaviour sections of the following characteristic sections and the other modified characteristic sections
        if difference!= 0
            allBs=["starting", "cruisingBeforeAcceleration", "acceleration", "cruising", "coasting", "cruisingAfterCoasting", "braking"]
            for csId in csIdMax+1:length(movingSectionOriginal.characteristicSections)
                for bs in 1: length(allBs)
                    if haskey(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs])
                        for point in 1:length(get(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs], BehaviorSection()).waypoints)
                            get(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]=get(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]+difference
                        end
                    end #if
                end #for
            end #for

            # updating the waypoints in the following modified charateristic sections and the following points in the driving course
            for modNr in csIdMax+1:length(energySavingModificationsWithCoasting)
                if energySavingModificationsWithCoasting[modNr].ratio>0
                    # updating the behavior sections of the modified charateristic section
                    for bs in 1: length(allBs)
                        if haskey(energySavingModificationsWithCoasting[modNr].csModified.behaviorSections, allBs[bs])
                            for point in 1:length(get(energySavingModificationsWithCoasting[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints)
                                get(energySavingModificationsWithCoasting[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]=get(energySavingModificationsWithCoasting[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]+difference
                            end
                        end #if
                    end #for

                    # correctig the points of the modified driving course
                    drivingCourseModifiedNew=Vector{Waypoint}()
                    for i in 1:endOfModificationId
                        push!(drivingCourseModifiedNew, Waypoint(drivingCourseNew[i]))
                    end # while
                    i=lastIdOfModifiedCs+1
                    while (i<=length(energySavingModificationsWithCoasting[modNr].drivingCourseModified))
                        push!(drivingCourseModifiedNew, Waypoint(energySavingModificationsWithCoasting[modNr].drivingCourseModified[i]))
                        drivingCourseModifiedNew[end].i=length(drivingCourseModifiedNew)
                        drivingCourseModifiedNew[end].t=drivingCourseModifiedNew[end-1].t+drivingCourseModifiedNew[end].Δt
                        drivingCourseModifiedNew[end].E=drivingCourseModifiedNew[end-1].E+drivingCourseModifiedNew[end].ΔE
                        drivingCourseModifiedNew[end].W_T=drivingCourseModifiedNew[end-1].W_T+drivingCourseModifiedNew[end].ΔW_T
                        i=i+1
                    end # while

                    energySavingModificationsWithCoasting[modNr].drivingCourseModified=drivingCourseModifiedNew
                end #if
            end #for

            for modNr in csIdMax+1:length(energySavingModificationsWithMaximumSpeed)
                if energySavingModificationsWithMaximumSpeed[modNr].ratio>0
                    # updating the behavior sections of the modified charateristic section
                    for bs in 1: length(allBs)
                        if haskey(energySavingModificationsWithMaximumSpeed[modNr].csModified.behaviorSections, allBs[bs])
                            for point in 1:length(get(energySavingModificationsWithMaximumSpeed[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints)
                                get(energySavingModificationsWithMaximumSpeed[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]=get(energySavingModificationsWithMaximumSpeed[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]+difference
                            end
                        end #if
                    end #for

                    # correctig the points of the modified driving course
                    drivingCourseModifiedNew=Vector{Waypoint}()
                    for i in 1:endOfModificationId
                        push!(drivingCourseModifiedNew, Waypoint(drivingCourseNew[i]))
                    end # while
                    i=lastIdOfModifiedCs+1
                    while (i<=length(energySavingModificationsWithMaximumSpeed[modNr].drivingCourseModified))
                        push!(drivingCourseModifiedNew, Waypoint(energySavingModificationsWithMaximumSpeed[modNr].drivingCourseModified[i]))
                        drivingCourseModifiedNew[end].i=length(drivingCourseModifiedNew)
                        drivingCourseModifiedNew[end].t=drivingCourseModifiedNew[end-1].t+drivingCourseModifiedNew[end].Δt
                        drivingCourseModifiedNew[end].E=drivingCourseModifiedNew[end-1].E+drivingCourseModifiedNew[end].ΔE
                        drivingCourseModifiedNew[end].W_T=drivingCourseModifiedNew[end-1].W_T+drivingCourseModifiedNew[end].ΔW_T
                        i=i+1
                    end # while

                    energySavingModificationsWithMaximumSpeed[modNr].drivingCourseModified=drivingCourseModifiedNew
                end #if
            end #for


            for modNr in csIdMax+1:length(energySavingModificationsWithCombination)
                if energySavingModificationsWithCombination[modNr].ratio>0
                    # updating the behavior sections of the modified charateristic section
                    for bs in 1: length(allBs)
                        if haskey(energySavingModificationsWithCombination[modNr].csModified.behaviorSections, allBs[bs])
                            for point in 1:length(get(energySavingModificationsWithCombination[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints)
                                get(energySavingModificationsWithCombination[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]=get(energySavingModificationsWithCombination[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).waypoints[point]+difference
                            end
                        end #if
                    end #for

                    # updating the points of the modified driving course
                    drivingCourseModifiedNew=Vector{Waypoint}()
                    for i in 1:endOfModificationId
                        push!(drivingCourseModifiedNew, Waypoint(drivingCourseNew[i]))
                    end # while
                    i=lastIdOfModifiedCs+1
                    while (i<=length(energySavingModificationsWithCombination[modNr].drivingCourseModified))
                        push!(drivingCourseModifiedNew, Waypoint(energySavingModificationsWithCombination[modNr].drivingCourseModified[i]))
                        drivingCourseModifiedNew[end].i=length(drivingCourseModifiedNew)
                        drivingCourseModifiedNew[end].t=drivingCourseModifiedNew[end-1].t+drivingCourseModifiedNew[end].Δt
                        drivingCourseModifiedNew[end].E=drivingCourseModifiedNew[end-1].E+drivingCourseModifiedNew[end].ΔE
                        drivingCourseModifiedNew[end].W_T=drivingCourseModifiedNew[end-1].W_T+drivingCourseModifiedNew[end].ΔW_T
                        i=i+1
                    end # while

                    energySavingModificationsWithCombination[modNr].drivingCourseModified=drivingCourseModifiedNew
                end #if
            end #for
        end # if difference


        drivingCourseOriginal=drivingCourseNew
        movingSectionOriginal.characteristicSections[csIdMax]=CharacteristicSection(movingSectionOriginal.energySavingModifications[end].csModified)

        movingSectionOriginal.t_total=drivingCourseOriginal[end].t            # total running time (in s)
        movingSectionOriginal.E_total=drivingCourseOriginal[end].E            # total energy consumption (in Ws)


        # modify new CS for the considered methods
        # method 1: increase coasting
        if doMethod1==true
            (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=increaseCostingSection(movingSectionOriginal.characteristicSections[csIdMax], drivingCourseOriginal, settings, vehicle, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
            if new
                energySavingModification=EnergySavingModification()
                energySavingModification.csId=csIdMax                                                                                                       # identifier of the characteristic section
                energySavingModification.type="increasing coasting"                                                                                         # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
                energySavingModification.csModified=characteristicSectionModified                                                                           # the modified characteristic section
                energySavingModification.drivingCourseModified=drivingCourseModifiedUntilEndOfModifiedCS                                                    # drivingCourse of the modified characteristic section
                energySavingModification.ΔE=movingSectionOriginal.characteristicSections[csIdMax].E_total-energySavingModification.csModified.E_total       # saved energy (in Ws)
                energySavingModification.Δt=energySavingModification.csModified.t_total-movingSectionOriginal.characteristicSections[csIdMax].t_total       # time loss (in s)
                if energySavingModification.Δt<=movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE>=0.0 #&& energySavingModification.Δt>0.0
                    energySavingModification.ratio=energySavingModification.ΔE/energySavingModification.Δt                          # ratio of ΔE and Δt (in Ws/s)
                elseif energySavingModification.Δt==0.0
                    energySavingModification.ratio=energySavingModification.ΔE/0.000000001
                else # Δt is to high
                    energySavingModification.ratio=0.0
                end
                energySavingModificationsWithCoasting[csIdMax]=energySavingModification
            else # there is no modification possible for the current CS
                energySavingModificationsWithCoasting[csIdMax]=EnergySavingModification()
            end
        end #if if doMethod1

        # method 2: accelerate to a lower v_reach
        if doMethod2==true
            (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=decreaseMaximumVelocity(movingSectionOriginal.characteristicSections[csIdMax], drivingCourseOriginal, settings, vehicle, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
            if new
                energySavingModification=EnergySavingModification()
                energySavingModification.csId=csIdMax                                                                               # identifier of the characteristic section
                energySavingModification.type="decreasing maximum velocity"                                                         # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
                energySavingModification.csModified=characteristicSectionModified                                                   # the modified characteristic section
                energySavingModification.drivingCourseModified=drivingCourseModifiedUntilEndOfModifiedCS                            # drivingCourse of the modified characteristic section
                energySavingModification.ΔE=movingSectionOriginal.characteristicSections[csIdMax].E_total-energySavingModification.csModified.E_total       # saved energy (in Ws)
                energySavingModification.Δt=energySavingModification.csModified.t_total-movingSectionOriginal.characteristicSections[csIdMax].t_total       # time loss (in s)
                if energySavingModification.Δt<=movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE>=0.0
                    energySavingModification.ratio=energySavingModification.ΔE/energySavingModification.Δt                          # ratio of ΔE and Δt (in Ws/s)
                elseif energySavingModification.Δt==0.0
                    energySavingModification.ratio=energySavingModification.ΔE/0.000000001
                else # Δt is to high
                    energySavingModification.ratio=0.0
                end
                energySavingModificationsWithMaximumSpeed[csIdMax]=energySavingModification
            else # there is no modification possible for the current CS
                energySavingModificationsWithMaximumSpeed[csIdMax]=EnergySavingModification()
            end
        end #if if doMethod


        # combination of both methods
        if doCombinationOfMethods==true
            (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=combineEnergySavingMethods(movingSectionOriginal.characteristicSections[csIdMax], drivingCourseOriginal, settings, vehicle, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
            if new
                energySavingModification=EnergySavingModification()
                energySavingModification.csId=csIdMax                                                                                                   # identifier of the characteristic section
                energySavingModification.type="combination of energy saving methods"                                                 # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
                energySavingModification.csModified=characteristicSectionModified                                                                       # the modified characteristic section
                energySavingModification.drivingCourseModified=drivingCourseModifiedUntilEndOfModifiedCS                                                # drivingCourse of the modified characteristic section
                energySavingModification.ΔE=movingSectionOriginal.characteristicSections[csIdMax].E_total-energySavingModification.csModified.E_total   # saved energy (in Ws)
                energySavingModification.Δt=energySavingModification.csModified.t_total-movingSectionOriginal.characteristicSections[csIdMax].t_total   # time loss (in s)
                if energySavingModification.Δt<=movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE>=0.0
                    energySavingModification.ratio=energySavingModification.ΔE/energySavingModification.Δt                                              # ratio of ΔE and Δt (in Ws/s)
                elseif energySavingModification.Δt==0.0 #
                    println("Energiesparanpassung durch Kombination bei CS ",csIdMax," ist 0 s. Das kann eigentlich nicht sein!")
                    energySavingModification.ratio=energySavingModification.ΔE/0.000000001
                else # Δt is to high
                    energySavingModification.ratio=0.0
                end
                energySavingModificationsWithCombination[csIdMax]=energySavingModification
            else # there is no modification possible for the current CS
                energySavingModificationsWithCombination[csIdMax]=EnergySavingModification()
            end
        end #if doCombinationOfMethods
    end # while

 #   println("t_recoveryAvailable=",movingSectionOriginal.t_recoveryAvailable)
    return (movingSectionOriginal, drivingCourseOriginal)
end #function simulateMinimumEnergyConsumption

end #module RailwayDrivingDynamicsOperationModes
