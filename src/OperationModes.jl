module OperationModes

using ..types

include("./MovingPhases.jl")
include("./EnergySaving.jl")
using .MovingPhases
using .EnergySaving

export calculateMinimumRunningTime!, calculateMinimumEnergyConsumption

approximationLevel = 6      # TODO: define it in TrainRun and give it to each function?

 # calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime!(movingSection::Dict, settings::Dict, train::Dict)
    CSs::Vector{CharacteristicSection} = movingSection[:characteristicSections]

    startingPoint=DataPoint()
    startingPoint.i=1
    startingPoint.s=CSs[1].s_entry
    drivingCourse=[startingPoint]    # List of data points

    #    for CS in CSs
    for csId in 1:length(CSs)
    #    println("CS",csId)
        # check if the CS has a cruising section
        s_breakFree = get(CSs[csId].behaviorSections, :breakFree, BehaviorSection()).length
        s_clearing = get(CSs[csId].behaviorSections, :clearing, BehaviorSection()).length
        s_acceleration = get(CSs[csId].behaviorSections, :acceleration, BehaviorSection()).length
        s_braking = max(0.0, ceil((CSs[csId].v_exit^2-CSs[csId].v_target^2)/2/train[:a_braking], digits=approximationLevel))   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors

        # calculate the cruising sections length
        s_cruising=CSs[csId].length-s_breakFree-s_clearing-s_acceleration-s_braking

        # reset the moving section (MS), delete characteristic sections (CS) that were used during the preperation for setting v_entry, v_target and v_exit
        delete!(CSs[csId].behaviorSections, :breakFree)
        delete!(CSs[csId].behaviorSections, :clearing)
        delete!(CSs[csId].behaviorSections, :acceleration)
        delete!(CSs[csId].behaviorSections, :diminishing)
        delete!(CSs[csId].behaviorSections, :cruising)
        CSs[csId].E=0.0
        CSs[csId].t=0.0


        if s_clearing == CSs[csId].length
                # 09/06 TODO: thought about using "cruising" because it is used in EnergySaving and not clearing (CSs[csId], drivingCourse)=addCruisingPhase!(CSs[csId], drivingCourse, s_clearing, settings, train, CSs, "cruising")
            (CSs[csId], drivingCourse)=addCruisingPhase!(CSs[csId], drivingCourse, s_clearing, settings, train, CSs, "clearing")
        elseif s_cruising == CSs[csId].length
            (CSs[csId], drivingCourse)=addCruisingPhase!(CSs[csId], drivingCourse, s_cruising, settings, train, CSs, "cruising")
        elseif s_cruising > 0.0 || s_braking == 0.0
        # 09/21 elseif s_cruising > 0.0
        # 09/21 elseif s_cruising > 0.01 # if the cruising section is longer than 1 cm (because of rounding issues not >0.0)
            if drivingCourse[end].v < CSs[csId].v_target
                (CSs[csId], drivingCourse)=addAccelerationPhase!(CSs[csId], drivingCourse, settings, train, CSs)
            end #if

            if CSs[csId].s_exit-drivingCourse[end].s-max(0.0, (CSs[csId].v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking]) < -0.001   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors
                println("ERROR: After accelerating in CS ",csId," the braking distance is too short!")
                println("     before acceleration in CS",csId, "  with s=",drivingCourse[end].s,"  s_braking=",((CSs[csId].v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking]),"   s_exit=",CSs[csId].s_exit)
                println("                             and v=",drivingCourse[end].v,"   v_target=",CSs[csId].v_target,"  v_exit=",CSs[csId].v_exit)
            end

            s_braking=max(0.0, ceil((CSs[csId].v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors
            s_cruising=CSs[csId].s_exit-drivingCourse[end].s-s_braking

            if s_cruising > 0.0
                (CSs[csId], drivingCourse)=addCruisingPhase!(CSs[csId], drivingCourse, s_cruising, settings, train, CSs, "cruising")
            end
        else

            if CSs[csId].v_entry < CSs[csId].v_target || s_acceleration > 0.0 # or instead of " || s_acceleration > 0.0" use "v_entry <= v_target" or "v_i <= v_target"
            # 09/09 old (not sufficient for steep gradients): if CSs[csId].v_entry < CSs[csId].v_target
                (CSs[csId], drivingCourse)=addAccelerationPhaseUntilBraking!(CSs[csId], drivingCourse, settings, train, CSs)
            end #if
        end #if


        s_braking=max(0.0, ceil((CSs[csId].v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))     # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors

        if drivingCourse[end].v > CSs[csId].v_exit
            #(CSs[csId], drivingCourse)=addBrakingPhase!(CSs[csId], drivingCourse, settings[:massModel], train, CSs)
            (CSs[csId], drivingCourse)=addBrakingPhase!(CSs[csId], drivingCourse, settings, train, CSs)
        end #if

        #= 09/20 old and should never be used:
        if drivingCourse[end].s < CSs[csId].s_exit
            if haskey(CSs[csId].behaviorSections, :cruising)
                println("INFO: A second cruising section has been added to CS ", csId," from s=",drivingCourse[end].s,"  to s_exit=",CSs[csId].s_exit)
            end
            (CSs[csId], drivingCourse)=addCruisingPhase!(CSs[csId], drivingCourse, s_cruising, settings, train, CSs, "cruising")
        end =#
    end #for

    (CSs[end], drivingCourse)=addStandstill!(CSs[end], drivingCourse, settings, train, CSs)

    movingSection[:t] = drivingCourse[end].t            # total running time (in s)
    movingSection[:E] = drivingCourse[end].E            # total energy consumption (in Ws)

    return (movingSection, drivingCourse)
end #function calculateMinimumRunningTime


function calculateMinimumEnergyConsumption(movingSectionMinimumRunningTime::Dict, drivingCourseMinimumRunningTime::Vector{DataPoint}, settings::Dict, train::Dict)
 # calculate a train run focussing on using the minimum possible energy consumption
    # booleans for choosing which methods are used for saving energy
    doMethod1=true
    #doMethod1=false
    #doMethod2=false
    doMethod2=true
    doCombinationOfMethods=true
    #doCombinationOfMethods=false

    #create a new moving section for the minimum energy consumption
    movingSectionOriginal=copyMovingSection(movingSectionMinimumRunningTime)
    CSsOrig::Vector{CharacteristicSection} = movingSectionOriginal[:characteristicSections]

    # create a new driving course for the minimum energy consumption
    drivingCourseOriginal=DataPoint[]
    for i in 1:length(drivingCourseMinimumRunningTime)
        push!(drivingCourseOriginal, DataPoint(drivingCourseMinimumRunningTime[i]))    # List of data points till the start of energy saving
    end

    # calculate the recovery time
    movingSectionOriginal[:t_recovery]=calculateRecoveryTime(movingSectionOriginal[:length], movingSectionOriginal[:t], train)
    movingSectionOriginal[:t_recoveryAvailable]=movingSectionOriginal[:t_recovery]

    # create arrays for each method with all the available energy saving modifications
    energySavingModificationsWithCoasting=EnergySavingModification[]
    energySavingModificationsWithMaximumSpeed=EnergySavingModification[]
    energySavingModificationsWithCombination=EnergySavingModification[]

    for csId in 1:length(CSsOrig)
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

    while movingSectionOriginal[:t_recoveryAvailable] > 0.0
        # comparison of modifications
        ratioMax=0.0
        csIdMax=0
        typeMax="none"
        (energySavingModificationsWithMaximumSpeed, ratioMax, csIdMax, typeMax) = findBestModification(energySavingModificationsWithMaximumSpeed, ratioMax, csIdMax, typeMax, movingSectionOriginal[:t_recoveryAvailable])
        (energySavingModificationsWithCoasting, ratioMax, csIdMax, typeMax)     = findBestModification(energySavingModificationsWithCoasting, ratioMax, csIdMax, typeMax, movingSectionOriginal[:t_recoveryAvailable])
        (energySavingModificationsWithCombination, ratioMax, csIdMax, typeMax)  = findBestModification(energySavingModificationsWithCombination, ratioMax, csIdMax, typeMax, movingSectionOriginal[:t_recoveryAvailable])


        # select the most efficient modification and update the original characteristicSection, drivingCourse and movingSection
        # in case none of the modifications has a ratio>0 stop the calculation
        if typeMax=="none"
             break
        elseif typeMax=="increasing coasting"
        #    println("Energy saving modification number ",length(movingSectionOriginal[:energySavingModifications])+1," (coasting) in CS ",csIdMax ," Δt=",energySavingModificationsWithCoasting[csIdMax].Δt," ΔE=", energySavingModificationsWithCoasting[csIdMax].ΔE," t_recoveryAvailable=", movingSectionOriginal[:t_recoveryAvailable]-energySavingModificationsWithCoasting[csIdMax].Δt)
            push!(movingSectionOriginal[:energySavingModifications], energySavingModificationsWithCoasting[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal[:energySavingModifications]),": typeMax=increasing coasting")
        elseif typeMax=="decreasing maximum velocity"
            push!(movingSectionOriginal[:energySavingModifications], energySavingModificationsWithMaximumSpeed[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal[:energySavingModifications]),": typeMax=decreasing maximum velocity")
        elseif typeMax=="combination of energy saving methods"
            push!(movingSectionOriginal[:energySavingModifications], energySavingModificationsWithCombination[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal[:energySavingModifications]),": typeMax=combination of energy saving methods")
        end #if

        movingSectionOriginal[:t_recoveryAvailable] = movingSectionOriginal[:t_recoveryAvailable] - movingSectionOriginal[:energySavingModifications][end].Δt

        lastIdOfSelectedCsOriginal = get(CSsOrig[csIdMax].behaviorSections, :standstill,
                                      get(CSsOrig[csIdMax].behaviorSections, :braking,
                                       get(CSsOrig[csIdMax].behaviorSections, :cruisingAfterCoasting,
                                        get(CSsOrig[csIdMax].behaviorSections, :coasting,
                                         get(CSsOrig[csIdMax].behaviorSections, :cruising,
                                          get(CSsOrig[csIdMax].behaviorSections, :acceleration,
                                           get(CSsOrig[csIdMax].behaviorSections, :clearing,
                                            get(CSsOrig[csIdMax].behaviorSections, :breakFree,
                                             get(CSsOrig[csIdMax].behaviorSections, :diminishing,
                                              BehaviorSection()))))))))).dataPoints[end]

        # if there is a diminishing phase its location must be analysed seperately because it could be before acceleration, between acceleration and cruising or after cruising. All the other behavior sections occur in a fixed order.
        if haskey(CSsOrig[csIdMax].behaviorSections, :diminishing)
            lastIdOfSelectedCsOriginal = max(lastIdOfSelectedCsOriginal, CSsOrig[csIdMax].behaviorSections[:diminishing].dataPoints[end])
        end

        # create new driving course
        drivingCourseNew=Vector{DataPoint}()
        for i in 1:length(movingSectionOriginal[:energySavingModifications][end].drivingCourseModified)
            push!(drivingCourseNew, DataPoint(movingSectionOriginal[:energySavingModifications][end].drivingCourseModified[i]))
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
        CSsOrig[csIdMax]=CharacteristicSection(movingSectionOriginal[:energySavingModifications][end].csModified)
        movingSectionOriginal[:t]=drivingCourseOriginal[end].t            # total running time (in s)
        movingSectionOriginal[:E]=drivingCourseOriginal[end].E            # total energy consumption (in Ws)

        # update all the data point references in the behaviour sections of the following characteristic sections and the other modified characteristic sections
        if difference!= 0
            # update the data point references in the behaviour sections of the following characteristic sections
            allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :cruisingAfterCoasting, :braking, :standstill]
            for csId in csIdMax+1:length(CSsOrig)
                for bs in 1: length(allBs)
                    if haskey(CSsOrig[csId].behaviorSections, allBs[bs])
                        for point in 1:length(CSsOrig[csId].behaviorSections[allBs[bs]].dataPoints)
                            CSsOrig[csId].behaviorSections[allBs[bs]].dataPoints[point] = CSsOrig[csId].behaviorSections[allBs[bs]].dataPoints[point]+difference
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

    (CSsOrig[end], drivingCourseOriginal) = addStandstill!(CSsOrig[end], drivingCourseOriginal, settings, train, CSsOrig)

    println("t_recoveryAvailable=",movingSectionOriginal[:t_recoveryAvailable])
    return (movingSectionOriginal, drivingCourseOriginal)
end #function calculateMinimumEnergyConsumption


function modifyCs(movingSectionOriginal::Dict, drivingCourseOriginal::Vector{DataPoint}, csId::Integer, modificationType::String, settings::Dict, train::Dict)
    CSsOrig::Vector{CharacteristicSection} = movingSectionOriginal[:characteristicSections]

    if modificationType == "increasing coasting"
        # method 1: increase coasting
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=increaseCoastingSection(CSsOrig[csId], drivingCourseOriginal, settings, train, CSsOrig, movingSectionOriginal[:t_recoveryAvailable])
    elseif modificationType == "decreasing maximum velocity"
        # method 2: accelerate to a lower v_target
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=decreaseMaximumVelocity(CSsOrig[csId], drivingCourseOriginal, settings, train, CSsOrig, movingSectionOriginal[:t_recoveryAvailable])
    elseif modificationType == "combination of energy saving methods"
        # calculate the combination of the previous methods
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=combineEnergySavingMethods(CSsOrig[csId], drivingCourseOriginal, settings, train, CSsOrig, movingSectionOriginal[:t_recoveryAvailable])
    else
        return EnergySavingModification()
    end

    energySavingModification = EnergySavingModification()
    if new
        energySavingModification.csId = csId                                                                                                # identifier of the characteristic section
        energySavingModification.type = modificationType                                                                                    # type of energy saving modification: "increasing coasting" or "decreasing maximum velocity" or "combination of energy saving methods"
        energySavingModification.csModified = characteristicSectionModified                                                                       # the modified characteristic section
        energySavingModification.drivingCourseModified = drivingCourseModifiedUntilEndOfModifiedCS                                                # drivingCourse of the modified characteristic section
        energySavingModification.ΔE = CSsOrig[csId].E - energySavingModification.csModified.E      # saved energy (in Ws)
        energySavingModification.Δt = energySavingModification.csModified.t - CSsOrig[csId].t      # time loss (in s)
        if energySavingModification.Δt <= movingSectionOriginal[:t_recoveryAvailable] && energySavingModification.ΔE >= 0.0
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
    allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :cruisingAfterCoasting, :braking, :standstill]
    difference = endOfModificationId-lastIdOfSelectedCsOriginal
    for modNr in csIdMax+1:length(energySavingModifications)
        if energySavingModifications[modNr].ratio>0
            # update the behavior sections of the modified charateristic section
            for bs in 1: length(allBs)
                if haskey(energySavingModifications[modNr].csModified.behaviorSections, allBs[bs])
                    for point in 1:length(energySavingModifications[modNr].csModified.behaviorSections[allBs[bs]].dataPoints)
                        energySavingModifications[modNr].csModified.behaviorSections[allBs[bs]].dataPoints[point] = energySavingModifications[modNr].csModified.behaviorSections[allBs[bs]].dataPoints[point]+difference
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

function copyMovingSection(original::Dict)
    CSsCopy = Vector{CharacteristicSection}()
    for csId in 1:length(original[:characteristicSections])
        push!(CSsCopy, CharacteristicSection(original[:characteristicSections][csId]))
    end #for

    ModificationsCopy = Vector{EnergySavingModification}()
    for modId in 1:length(original[:energySavingModifications])
        push!(ModificationsCopy, EnergySavingModification(original[:energySavingModifications][modId]))
    end #for

    copy = Dict(:id => original[:id],                                       # identifier
                :length => original[:length],                               # total length (in m)
                :s_entry => original[:s_entry],                             # first position (in m)
                :s_exit => original[:s_exit],                               # last position (in m)
                :t => original[:t],                                         # total running time (in s)
                :E => original[:E],                                         # total energy consumption (in Ws)
                :t_recovery => original[:t_recovery],                       # total recovery time for energy-saving modifications (in s)
                :t_recoveryAvailable => original[:t_recoveryAvailable],     # still available recovery time for energy-saving modifications (in s)
                :characteristicSections => CSsCopy,                         # list of containing characteristic sections
                :energySavingModifications => ModificationsCopy)            # list of containing all the used energy saving modifications
    return copy
end #function copyMovingSection

end #module OperationModes
