module OperationModes

using ..types

include("./MovingPhases.jl")
include("./EnergySaving.jl")
using .MovingPhases
using .EnergySaving

export calculateMinimumRunningTime!, calculateMinimumEnergyConsumption

approximationLevel = 6      # TODO: define it in TrainRun and give it to each function?

 # calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime!(movingSection::MovingSection, settings::Settings, train::Train)
# CSs=movingSection.characteristicSections
    startingPoint=DataPoint()
    startingPoint.i=1
    startingPoint.s=movingSection.characteristicSections[1].s_entry
    drivingCourse=[startingPoint]    # List of data points

    #    for CS in CSs
    for csId in 1:length(movingSection.characteristicSections)
    #    println("CS",csId)
        # check if the CS has a cruising section
        s_breakFree=get(movingSection.characteristicSections[csId].behaviorSections, "breakFree", BehaviorSection()).length
        s_clearing=get(movingSection.characteristicSections[csId].behaviorSections, "clearing", BehaviorSection()).length
        s_acceleration=get(movingSection.characteristicSections[csId].behaviorSections, "acceleration", BehaviorSection()).length
        s_braking=max(0.0, ceil((movingSection.characteristicSections[csId].v_exit^2-movingSection.characteristicSections[csId].v_target^2)/2/train.a_braking, digits=approximationLevel))   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors

        # calculate the cruising sections length
        s_cruising=movingSection.characteristicSections[csId].length-s_breakFree-s_clearing-s_acceleration-s_braking

        # reset the moving section (MS), delete characteristic sections (CS) that were used during the preperation for setting v_entry, v_target and v_exit
        delete!(movingSection.characteristicSections[csId].behaviorSections, "breakFree")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "clearing")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "acceleration")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "diminishing")
        delete!(movingSection.characteristicSections[csId].behaviorSections, "cruising")
        movingSection.characteristicSections[csId].E=0.0
        movingSection.characteristicSections[csId].t=0.0


        if s_clearing == movingSection.characteristicSections[csId].length
                # 09/06 TODO: thought about using "cruising" because it is used in EnergySaving and not clearing (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_clearing, settings, train, movingSection.characteristicSections, "cruising")
            (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_clearing, settings, train, movingSection.characteristicSections, "clearing")
        elseif s_cruising == movingSection.characteristicSections[csId].length
            (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruising, settings, train, movingSection.characteristicSections, "cruising")
        elseif s_cruising > 0.0 || s_braking == 0.0
        # 09/21 elseif s_cruising > 0.0
        # 09/21 elseif s_cruising > 0.01 # if the cruising section is longer than 1 cm (because of rounding issues not >0.0)
            if drivingCourse[end].v < movingSection.characteristicSections[csId].v_target
                (movingSection.characteristicSections[csId], drivingCourse)=addAccelerationPhase!(movingSection.characteristicSections[csId], drivingCourse, settings, train, movingSection.characteristicSections)
            end #if

            if movingSection.characteristicSections[csId].s_exit-drivingCourse[end].s-max(0.0, (movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/train.a_braking) < -0.001   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors
                println("ERROR: After accelerating in CS ",csId," the braking distance is too short!")
                println("     before acceleration in CS",csId, "  with s=",drivingCourse[end].s,"  s_braking=",((movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/train.a_braking),"   s_exit=",movingSection.characteristicSections[csId].s_exit)
                println("                             and v=",drivingCourse[end].v,"   v_target=",movingSection.characteristicSections[csId].v_target,"  v_exit=",movingSection.characteristicSections[csId].v_exit)
            end

            s_braking=max(0.0, ceil((movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/train.a_braking, digits=approximationLevel))   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors
            s_cruising=movingSection.characteristicSections[csId].s_exit-drivingCourse[end].s-s_braking

            if s_cruising > 0.0
                (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruising, settings, train, movingSection.characteristicSections, "cruising")
            end
        else

            if movingSection.characteristicSections[csId].v_entry < movingSection.characteristicSections[csId].v_target || s_acceleration > 0.0 # or instead of " || s_acceleration > 0.0" use "v_entry <= v_target" or "v_i <= v_target"
            # 09/09 old (not sufficient for steep gradients): if movingSection.characteristicSections[csId].v_entry < movingSection.characteristicSections[csId].v_target
                (movingSection.characteristicSections[csId], drivingCourse)=addAccelerationPhaseUntilBraking!(movingSection.characteristicSections[csId], drivingCourse, settings, train, movingSection.characteristicSections)
            end #if
        end #if


        s_braking=max(0.0, ceil((movingSection.characteristicSections[csId].v_exit^2-drivingCourse[end].v^2)/2/train.a_braking, digits=approximationLevel))     # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors

        if drivingCourse[end].v > movingSection.characteristicSections[csId].v_exit
            #(movingSection.characteristicSections[csId], drivingCourse)=addBrakingPhase!(movingSection.characteristicSections[csId], drivingCourse, settings.massModel, train, movingSection.characteristicSections)
            (movingSection.characteristicSections[csId], drivingCourse)=addBrakingPhase!(movingSection.characteristicSections[csId], drivingCourse, settings, train, movingSection.characteristicSections)
        end #if

        #= 09/20 old and should never be used:
        if drivingCourse[end].s < movingSection.characteristicSections[csId].s_exit
            if haskey(movingSection.characteristicSections[csId].behaviorSections, "cruising")
                println("INFO: A second cruising section has been added to CS ", csId," from s=",drivingCourse[end].s,"  to s_exit=",movingSection.characteristicSections[csId].s_exit)
            end
            (movingSection.characteristicSections[csId], drivingCourse)=addCruisingPhase!(movingSection.characteristicSections[csId], drivingCourse, s_cruising, settings, train, movingSection.characteristicSections, "cruising")
        end =#
    end #for

    # calculate the last data points resiting forces
    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings.massModel,  movingSection.characteristicSections, "braking"))

    movingSection.t=drivingCourse[end].t            # total running time (in s)
    movingSection.E=drivingCourse[end].E            # total energy consumption (in Ws)

    return (movingSection, drivingCourse)
end #function calculateMinimumRunningTime


function calculateMinimumEnergyConsumption(movingSectionMinimumRunningTime::MovingSection, drivingCourseMinimumRunningTime::Vector{DataPoint}, settings::Settings, train::Train)
 # calculate a train run focussing on using the minimum possible energy consumption
    # booleans for choosing which methods are used for saving energy
    doMethod1=true
    #doMethod1=false
    #doMethod2=false
    doMethod2=true
    doCombinationOfMethods=true
    #doCombinationOfMethods=false

    #create a new moving section for the minimum energy consumption
    movingSectionOriginal=MovingSection(movingSectionMinimumRunningTime)

    # create a new driving course for the minimum energy consumption
    drivingCourseOriginal=DataPoint[]
    for i in 1:length(drivingCourseMinimumRunningTime)
        push!(drivingCourseOriginal, DataPoint(drivingCourseMinimumRunningTime[i]))    # List of data points till the start of energy saving
    end

    # calculate the recovery time
    movingSectionOriginal.t_recovery=calculateRecoveryTime(movingSectionOriginal.length, movingSectionOriginal.t, train)
    movingSectionOriginal.t_recoveryAvailable=movingSectionOriginal.t_recovery

    # create arrays for each method with all the available energy saving modifications
    energySavingModificationsWithCoasting=EnergySavingModification[]
    energySavingModificationsWithMaximumSpeed=EnergySavingModification[]
    energySavingModificationsWithCombination=EnergySavingModification[]

    for csId in 1:length(movingSectionOriginal.characteristicSections)
        # method 1: increase coasting
        if doMethod1 == true
            modificationType = "increasing coasting"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csId, modificationType, settings, train)
            push!(energySavingModificationsWithCoasting, energySavingModification)
        end #if doMethod1

        # method 2: accelerate to a lower v_target
        if doMethod2 == true
            modificationType = "decreasing maximum velocity"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csId, modificationType, settings, train)
            push!(energySavingModificationsWithMaximumSpeed, energySavingModification)
        end #if doMethod2

        # calculate the combination of the previous methods
        if doCombinationOfMethods == true
            modificationType = "combination of energy saving methods"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csId, modificationType, settings, train)
            push!(energySavingModificationsWithCombination, energySavingModification)
        end #if
    end # for

    while movingSectionOriginal.t_recoveryAvailable > 0.0
        # comparison of modifications
        ratioMax=0.0
        csIdMax=0
        typeMax="none"
        (energySavingModificationsWithMaximumSpeed, ratioMax, csIdMax, typeMax) = findBestModification(energySavingModificationsWithMaximumSpeed, ratioMax, csIdMax, typeMax, movingSectionOriginal.t_recoveryAvailable)
        # 09/14 Three if cases for testing:
#=        if length(movingSectionOriginal.energySavingModifications)==895 && ratioMax>0.0
            println("")
            println("typeMax=",typeMax)
            println("ratioMax=",ratioMax)
            println("csIdMax=",csIdMax)
            println("---")


            println("type=",energySavingModificationsWithMaximumSpeed[csIdMax].type)
            println("  csId=",energySavingModificationsWithMaximumSpeed[csIdMax].csId)
            println("  ΔE=",energySavingModificationsWithMaximumSpeed[csIdMax].ΔE)
            println("  Δt=",energySavingModificationsWithMaximumSpeed[csIdMax].Δt)
            println("  ratio=",energySavingModificationsWithMaximumSpeed[csIdMax].ratio)
            println("  DC-length=",length(energySavingModificationsWithMaximumSpeed[csIdMax].drivingCourseModified))
        end
=#
        (energySavingModificationsWithCoasting, ratioMax, csIdMax, typeMax)     = findBestModification(energySavingModificationsWithCoasting, ratioMax, csIdMax, typeMax, movingSectionOriginal.t_recoveryAvailable)
#=        if length(movingSectionOriginal.energySavingModifications)==895 && ratioMax>0.0
            println("typeMax=",typeMax)
            println("ratioMax=",ratioMax)
            println("csIdMax=",csIdMax)
            println("---")


            println("type=",energySavingModificationsWithCoasting[csIdMax].type)
            println("  csId=",energySavingModificationsWithCoasting[csIdMax].csId)
            println("  ΔE=",energySavingModificationsWithCoasting[csIdMax].ΔE)
            println("  Δt=",energySavingModificationsWithCoasting[csIdMax].Δt)
            println("  ratio=",energySavingModificationsWithCoasting[csIdMax].ratio)
            println("  DC-length=",length(energySavingModificationsWithCoasting[csIdMax].drivingCourseModified))
        end

=#
        (energySavingModificationsWithCombination, ratioMax, csIdMax, typeMax)  = findBestModification(energySavingModificationsWithCombination, ratioMax, csIdMax, typeMax, movingSectionOriginal.t_recoveryAvailable)
#=        if length(movingSectionOriginal.energySavingModifications)==895 && ratioMax>0.0
            println("typeMax=",typeMax)
            println("ratioMax=",ratioMax)
            println("csIdMax=",csIdMax)
            println("---")


            println("type=",energySavingModificationsWithCombination[csIdMax].type)
            println("  csId=",energySavingModificationsWithCombination[csIdMax].csId)
            println("  ΔE=",energySavingModificationsWithCombination[csIdMax].ΔE)
            println("  Δt=",energySavingModificationsWithCombination[csIdMax].Δt)
            println("  ratio=",energySavingModificationsWithCombination[csIdMax].ratio)
            println("  DC-length=",length(energySavingModificationsWithCombination[csIdMax].drivingCourseModified))
        end
=#


        # select the most efficient modification and update the original characteristicSection, drivingCourse and movingSection
        # in case none of the modifications has a ratio>0 stop the calculation
        if typeMax=="none"
             break
        elseif typeMax=="increasing coasting"
        #    println("Energy saving modification number ",length(movingSectionOriginal.energySavingModifications)+1," (coasting) in CS ",csIdMax ," Δt=",energySavingModificationsWithCoasting[csIdMax].Δt," ΔE=", energySavingModificationsWithCoasting[csIdMax].ΔE," t_recoveryAvailable=", movingSectionOriginal.t_recoveryAvailable-energySavingModificationsWithCoasting[csIdMax].Δt)
            push!(movingSectionOriginal.energySavingModifications, energySavingModificationsWithCoasting[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal.energySavingModifications),": typeMax=increasing coasting")
        elseif typeMax=="decreasing maximum velocity"
            push!(movingSectionOriginal.energySavingModifications, energySavingModificationsWithMaximumSpeed[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal.energySavingModifications),": typeMax=decreasing maximum velocity")
        elseif typeMax=="combination of energy saving methods"
            push!(movingSectionOriginal.energySavingModifications, energySavingModificationsWithCombination[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal.energySavingModifications),": typeMax=combination of energy saving methods")
        end #if

        movingSectionOriginal.t_recoveryAvailable = movingSectionOriginal.t_recoveryAvailable - movingSectionOriginal.energySavingModifications[end].Δt

        lastIdOfSelectedCsOriginal = get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "braking", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "cruisingAfterCoasting", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "coasting", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "cruising", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "acceleration", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "clearing", get(movingSectionOriginal.characteristicSections[csIdMax].behaviorSections, "breakFree", BehaviorSection()))))))).dataPoints[end]

        # create new driving course
        drivingCourseNew=Vector{DataPoint}()
        for i in 1:length(movingSectionOriginal.energySavingModifications[end].drivingCourseModified)
            push!(drivingCourseNew, DataPoint(movingSectionOriginal.energySavingModifications[end].drivingCourseModified[i]))
        end

    #    # for testing if the modifications driving course and the original one have the same speed value at their transition point
    #    if lastIdOfSelectedCsOriginal+1<=drivingCourseOriginal[end].i
    #        if drivingCourseOriginal[lastIdOfSelectedCsOriginal].v-drivingCourseNew[end].v!=0.0
    #            println("INFO: Inconsistency while adding the CS",csIdMax," that has been modified for consuming less energy. The difference at its end is ",(drivingCourseOriginal[lastIdOfSelectedCsOriginal].v-drivingCourseNew[end].v)," m/s."
    #        end
    #    end # for testing

        # for testing 09/09
        if length(drivingCourseNew) == 0
        #=    println("typeMax=",typeMax)
            println("ratioMax=",ratioMax)
            println("csIdMax=",csIdMax)
            println("---")


            println("type=",energySavingModificationsWithCombination[csIdMax].type)
            println("  csId=",energySavingModificationsWithCombination[csIdMax].csId)
            println("  ΔE=",energySavingModificationsWithCombination[csIdMax].ΔE)
            println("  Δt=",energySavingModificationsWithCombination[csIdMax].Δt)
            println("  ratio=",energySavingModificationsWithCombination[csIdMax].ratio)
            println("  DC-length=",length(energySavingModificationsWithCombination[csIdMax].drivingCourseModified))
        =#
            println("---")
            println("energySavingModificationsWithCoasting:",length(energySavingModificationsWithCoasting[csIdMax].drivingCourseModified))
            println("energySavingModificationsWithMaximumSpeed:",length(energySavingModificationsWithMaximumSpeed[csIdMax].drivingCourseModified))
            println("energySavingModificationsWithCombination:",length(energySavingModificationsWithCombination[csIdMax].drivingCourseModified))
        end

        #fill up the rest of the driving course with information from the original course
        drivingCourseNew[end].F_T=drivingCourseOriginal[lastIdOfSelectedCsOriginal].F_T
        drivingCourseNew[end].R_traction=drivingCourseOriginal[lastIdOfSelectedCsOriginal].R_traction
        drivingCourseNew[end].R_consist=drivingCourseOriginal[lastIdOfSelectedCsOriginal].R_consist
        drivingCourseNew[end].R_train=drivingCourseOriginal[lastIdOfSelectedCsOriginal].R_train
        drivingCourseNew[end].R_path=drivingCourseOriginal[lastIdOfSelectedCsOriginal].R_path
        drivingCourseNew[end].F_R=drivingCourseOriginal[lastIdOfSelectedCsOriginal].F_R
        drivingCourseNew[end].a=drivingCourseOriginal[lastIdOfSelectedCsOriginal].a

        endOfModificationId=drivingCourseNew[end].i                 # is needed for updating the other modified driving courses
        difference=endOfModificationId-lastIdOfSelectedCsOriginal

        i=lastIdOfSelectedCsOriginal+1
        while i <= length(drivingCourseOriginal)
            push!(drivingCourseNew, DataPoint(drivingCourseOriginal[i]))
            drivingCourseNew[end].i=length(drivingCourseNew)
            drivingCourseNew[end].t=drivingCourseNew[end-1].t+drivingCourseNew[end].Δt
            drivingCourseNew[end].E=drivingCourseNew[end-1].E+drivingCourseNew[end].ΔE
            drivingCourseNew[end].W=drivingCourseNew[end-1].W+drivingCourseNew[end].ΔW
            i=i+1
        end # while

        # replace the original driving course and CS with the new modified ones
        drivingCourseOriginal=drivingCourseNew
        movingSectionOriginal.characteristicSections[csIdMax]=CharacteristicSection(movingSectionOriginal.energySavingModifications[end].csModified)
        movingSectionOriginal.t=drivingCourseOriginal[end].t            # total running time (in s)
        movingSectionOriginal.E=drivingCourseOriginal[end].E            # total energy consumption (in Ws)

        # update all the data point references in the behaviour sections of the following characteristic sections and the other modified characteristic sections
        if difference!= 0
            # update the data point references in the behaviour sections of the following characteristic sections
            allBs=["breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting","cruisingAfterCoasting", "braking", "standStill"]
            for csId in csIdMax+1:length(movingSectionOriginal.characteristicSections)
                for bs in 1: length(allBs)
                    if haskey(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs])
                        for point in 1:length(get(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs], BehaviorSection()).dataPoints)
                            get(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs], BehaviorSection()).dataPoints[point]=get(movingSectionOriginal.characteristicSections[csId].behaviorSections, allBs[bs], BehaviorSection()).dataPoints[point]+difference
                        end
                    end #if
                end #for
            end #for

            # update the data points in the following modified charateristic sections and the following points in the driving course
            energySavingModificationsWithCoasting = updateEnergySavingModifications(energySavingModificationsWithCoasting, csIdMax, drivingCourseNew, endOfModificationId, lastIdOfSelectedCsOriginal)
            energySavingModificationsWithMaximumSpeed = updateEnergySavingModifications(energySavingModificationsWithMaximumSpeed, csIdMax, drivingCourseNew, endOfModificationId, lastIdOfSelectedCsOriginal)
            energySavingModificationsWithCombination = updateEnergySavingModifications(energySavingModificationsWithCombination, csIdMax, drivingCourseNew, endOfModificationId, lastIdOfSelectedCsOriginal)
        end # if difference

        # modify new CS for the considered methods
        # method 1: increase coasting
        if doMethod1==true
            modificationType = "increasing coasting"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csIdMax, modificationType, settings, train)
            energySavingModificationsWithCoasting[csIdMax]=energySavingModification
        end #if if doMethod1

        # method 2: accelerate to a lower v_target
        if doMethod2==true
            modificationType = "decreasing maximum velocity"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csIdMax, modificationType, settings, train)
            energySavingModificationsWithMaximumSpeed[csIdMax]=energySavingModification
        end #if if doMethod

        # combination of both methods
        if doCombinationOfMethods==true
            modificationType = "combination of energy saving methods"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csIdMax, modificationType, settings, train)
            energySavingModificationsWithCombination[csIdMax]=energySavingModification
        end #if doCombinationOfMethods
    end # while


    println("t_recoveryAvailable=",movingSectionOriginal.t_recoveryAvailable)
    return (movingSectionOriginal, drivingCourseOriginal)
end #function calculateMinimumEnergyConsumption


function modifyCs(movingSectionOriginal::MovingSection, drivingCourseOriginal::Vector{DataPoint}, csId::Integer, modificationType::String, settings::Settings, train::Train)
    #println("drin: ",modificationType)
    if modificationType == "increasing coasting"
        # method 1: increase coasting
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=increaseCoastingSection(movingSectionOriginal.characteristicSections[csId], drivingCourseOriginal, settings, train, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
    elseif modificationType == "decreasing maximum velocity"
        # method 2: accelerate to a lower v_target
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=decreaseMaximumVelocity(movingSectionOriginal.characteristicSections[csId], drivingCourseOriginal, settings, train, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
    elseif modificationType == "combination of energy saving methods"
        # calculate the combination of the previous methods
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=combineEnergySavingMethods(movingSectionOriginal.characteristicSections[csId], drivingCourseOriginal, settings, train, movingSectionOriginal.characteristicSections, movingSectionOriginal.t_recoveryAvailable)
    else
        return EnergySavingModification()
    end

    energySavingModification = EnergySavingModification()
    if new
        energySavingModification.csId = csId                                                                                                # identifier of the characteristic section
        energySavingModification.type = modificationType                                                                                    # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
        energySavingModification.csModified = characteristicSectionModified                                                                       # the modified characteristic section
        energySavingModification.drivingCourseModified = drivingCourseModifiedUntilEndOfModifiedCS                                                # drivingCourse of the modified characteristic section
        energySavingModification.ΔE = movingSectionOriginal.characteristicSections[csId].E - energySavingModification.csModified.E      # saved energy (in Ws)
        energySavingModification.Δt = energySavingModification.csModified.t - movingSectionOriginal.characteristicSections[csId].t      # time loss (in s)
        if energySavingModification.Δt <= movingSectionOriginal.t_recoveryAvailable && energySavingModification.ΔE >= 0.0
            #*** TODO: check why "sign" is needed here
        #    if modificationType == "combination of energy saving methods"
                energySavingModification.ratio=sign(energySavingModification.Δt)*energySavingModification.ΔE/energySavingModification.Δt        # ratio of ΔE and Δt (in Ws/s)
        #    else
        #            energySavingModification.ratio = energySavingModification.ΔE / energySavingModification.Δt                                              # ratio of ΔE and Δt (in Ws/s)
        #    end
            # ***
        elseif energySavingModification.Δt == 0.0
            energySavingModification.ratio = energySavingModification.ΔE/0.000000001
        else # Δt is to high or ΔE < 0.0 Ws
            energySavingModification.ratio = 0.0
        end
    end
    return energySavingModification
end #function modifyCs


function findBestModification(energySavingModifications::Vector{EnergySavingModification}, ratioMax::AbstractFloat, csIdMax::Integer, typeMax::String, t_recoveryAvailable::AbstractFloat)
    for modNr in 1:length(energySavingModifications)
        if energySavingModifications[modNr].ratio > ratioMax
            if energySavingModifications[modNr].Δt <= t_recoveryAvailable
                ratioMax=energySavingModifications[modNr].ratio
                csIdMax=energySavingModifications[modNr].csId
                typeMax=energySavingModifications[modNr].type
            else # Δt is to high
                energySavingModifications[modNr].ratio=0.0
            end #if
        end #if
    end #for
    return (energySavingModifications, ratioMax, csIdMax, typeMax)
end #function findBestModification

function updateEnergySavingModifications(energySavingModifications::Vector{EnergySavingModification}, csIdMax::Integer, drivingCourseNew::Vector{DataPoint}, endOfModificationId::Integer, lastIdOfSelectedCsOriginal::Integer)
    allBs=["breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting","cruisingAfterCoasting", "braking", "standStill"]
    difference = endOfModificationId-lastIdOfSelectedCsOriginal
    for modNr in csIdMax+1:length(energySavingModifications)
        if energySavingModifications[modNr].ratio>0
            # update the behavior sections of the modified charateristic section
            for bs in 1: length(allBs)
                if haskey(energySavingModifications[modNr].csModified.behaviorSections, allBs[bs])
                    for point in 1:length(get(energySavingModifications[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).dataPoints)
                        get(energySavingModifications[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).dataPoints[point] = get(energySavingModifications[modNr].csModified.behaviorSections, allBs[bs], BehaviorSection()).dataPoints[point]+difference
                    end
                end #if
            end #for

            # correct the points of previous CS in the modified driving course. Copy the new driving course till the beginning of the current CS and change total values of the current modified CS data points accordingly
            drivingCourseModifiedNew=Vector{DataPoint}()
            for i in 1:endOfModificationId
                push!(drivingCourseModifiedNew, DataPoint(drivingCourseNew[i]))
            end # for
            i=lastIdOfSelectedCsOriginal+1
            while i <= length(energySavingModifications[modNr].drivingCourseModified)
                push!(drivingCourseModifiedNew, DataPoint(energySavingModifications[modNr].drivingCourseModified[i]))
                drivingCourseModifiedNew[end].i=length(drivingCourseModifiedNew)
                drivingCourseModifiedNew[end].t=drivingCourseModifiedNew[end-1].t+drivingCourseModifiedNew[end].Δt
                drivingCourseModifiedNew[end].E=drivingCourseModifiedNew[end-1].E+drivingCourseModifiedNew[end].ΔE
                drivingCourseModifiedNew[end].W=drivingCourseModifiedNew[end-1].W+drivingCourseModifiedNew[end].ΔW
                i=i+1
            end # while

            energySavingModifications[modNr].drivingCourseModified = drivingCourseModifiedNew
        end #if
    end #for

    return energySavingModifications
end #function updateEnergySavingModifications

end #module OperationModes
