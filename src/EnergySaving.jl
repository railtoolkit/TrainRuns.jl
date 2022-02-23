#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

# INFO: EnergySaving should not be used because it is not completed yet. It was used to show the possibility of calculating different operation modes.
# TODO: It has to be optimized so that each ernergy saving method is working individually for every train on every path.

# TODO: calculation time for passenger trains on path1 is very long and should be reduced
# TODO from 2022/01/18: Test if enum trainType is working correctly in function calculateRecoveryTime or if only the else-pathis taken
# TODO from 2022/01/19: Are here calculations that should be transferred to DrivingDynamics.jl?
# TODO from 2022/01/22: use always copyCharacteristicSection and don't do it manually like "csModified=Dict(:id => csOriginal[:id], ..." three times

module EnergySaving

# include modules of TrainRunCalc
include("./Behavior.jl")

# use modules of TrainRunCalc
using .Behavior

export addOperationModeEnergySaving!

@enum trainType passenger=1 freight=2 motorCoachTrain=3

approximationLevel = 6  # value for approximation to intersections
    # TODO: define it in TrainRun and give it to each function?

## functions for calculating the operation mode for the minimum energy consumption

# calculate the train run for operation mode "minimum energy consumption"
function addOperationModeEnergySaving!(accumulatedDict::Dict)
    if accumulatedDict[:settings][:operationModeMinimumEnergyConsumption] == true
        movingSectionMinimumRunningTime = accumulatedDict[:movingSectionMinimumRunningTime]
        drivingCourseMinimumRunningTime = accumulatedDict[:drivingCourseMinimumRunningTime]
        settings = accumulatedDict[:settings]
        train = accumulatedDict[:train]
        (movingSectionMinimumEnergyConsumption, drivingCourseMinimumEnergyConsumption)=calculateMinimumEnergyConsumption(movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime, settings, train)
        println("The driving course for the lowest energy consumption has been calculated.")

        # accumulate data and create an output dictionary
        merge!(accumulatedDict, Dict(:movingSectionMinimumEnergyConsumption => movingSectionMinimumEnergyConsumption, :drivingCourseMinimumEnergyConsumption => drivingCourseMinimumEnergyConsumption))
    else
        println("No output for minimum energy consumption has been demanded and so none will be calculated.")
    end #if

    return accumulatedDict
end #function addOperationModeEnergySaving!

function calculateMinimumEnergyConsumption(movingSectionMinimumRunningTime::Dict, drivingCourseMinimumRunningTime::Vector{Dict}, settings::Dict, train::Dict)
 # calculate a train run focussing on using the minimum possible energy consumption
    # booleans for choosing which methods are used for saving energy
    doMethod1=true
    #doMethod1=false

    doMethod2=true
    #doMethod2=false

    doCombinationOfMethods=true
    #doCombinationOfMethods=false

    # create a new driving course for the minimum energy consumption
    drivingCourseOriginal = copy(drivingCourseMinimumRunningTime)

    #create a new moving section for the minimum energy consumption
    movingSectionOriginal=copyMovingSection(movingSectionMinimumRunningTime)
    # 01/09 old not sure if just copy is enough.. : movingSectionOriginal=copy(movingSectionMinimumRunningTime)
    CSsOrig::Vector{Dict} = movingSectionOriginal[:characteristicSections]
    merge!(movingSectionOriginal, Dict(:energySavingModifications => []))   # list containing all the used energy saving modifications

    # calculate the recovery time
    t_recovery=calculateRecoveryTime(movingSectionOriginal[:length], movingSectionOriginal[:t], train)
    merge!(movingSectionOriginal, Dict(:t_recovery=>t_recovery))                    # total recovery time for energy-saving modifications (in s)
    merge!(movingSectionOriginal, Dict(:t_recoveryAvailable => t_recovery))         # still available recovery time for energy-saving modifications (in s) initialized with the total recovery time

    # create arrays for each method with all the available energy saving modifications
    energySavingModificationsWithCoasting=Dict[]
    energySavingModificationsWithMaximumSpeed=Dict[]
    energySavingModificationsWithCombination=Dict[]

    for csId in 1:length(CSsOrig)
        # method 1: increase coasting
        if doMethod1 == true
            modificationType = "increasing coasting"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csId, modificationType, settings, train)
            push!(energySavingModificationsWithCoasting, energySavingModification)
        end #if doMethod1

        # method 2: accelerate to a lower v_peak
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

    # 01/03 old wit too long calculation time: while movingSectionOriginal[:t_recoveryAvailable] > 0.0
    while movingSectionOriginal[:t_recoveryAvailable] >= 1/(10^approximationLevel)
        # compare modifications
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
        #    println("Energy saving modification number ",length(movingSectionOriginal[:energySavingModifications])+1," (coasting) in CS ",csIdMax ," Δt=",energySavingModificationsWithCoasting[csIdMax][:Δt]," ΔE=", energySavingModificationsWithCoasting[csIdMax][:ΔE]," t_recoveryAvailable=", movingSectionOriginal[:t_recoveryAvailable]-energySavingModificationsWithCoasting[csIdMax][:Δt])
            push!(movingSectionOriginal[:energySavingModifications], energySavingModificationsWithCoasting[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal[:energySavingModifications]),": typeMax=increasing coasting")
        elseif typeMax=="decreasing maximum velocity"
            push!(movingSectionOriginal[:energySavingModifications], energySavingModificationsWithMaximumSpeed[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal[:energySavingModifications]),": typeMax=decreasing maximum velocity")
        elseif typeMax=="combination of energy saving methods"
            push!(movingSectionOriginal[:energySavingModifications], energySavingModificationsWithCombination[csIdMax])
        #    println("Nr. ",length(movingSectionOriginal[:energySavingModifications]),": typeMax=combination of energy saving methods")
        end #if

        movingSectionOriginal[:t_recoveryAvailable] = movingSectionOriginal[:t_recoveryAvailable] - movingSectionOriginal[:energySavingModifications][end][:Δt]

        lastIdOfSelectedCsOriginal = get(CSsOrig[csIdMax][:behaviorSections], :standstill,
                                      get(CSsOrig[csIdMax][:behaviorSections], :braking,
                                        get(CSsOrig[csIdMax][:behaviorSections], :coasting,
                                         get(CSsOrig[csIdMax][:behaviorSections], :cruising,
                                          get(CSsOrig[csIdMax][:behaviorSections], :acceleration,
                                           get(CSsOrig[csIdMax][:behaviorSections], :clearing,
                                            get(CSsOrig[csIdMax][:behaviorSections], :breakFree,
                                             get(CSsOrig[csIdMax][:behaviorSections], :diminishing,
                                               Dict(:dataPoints => [0])))))))))[:dataPoints][end]

        # if there is a diminishing phase its location must be analysed seperately because it could be before acceleration, between acceleration and cruising or after cruising. All the other behavior sections occure in a fixed order.
        if haskey(CSsOrig[csIdMax][:behaviorSections], :diminishing)
            lastIdOfSelectedCsOriginal = max(lastIdOfSelectedCsOriginal, CSsOrig[csIdMax][:behaviorSections][:diminishing][:dataPoints][end])
        end

        # create new driving course
        drivingCourseNew = copy(movingSectionOriginal[:energySavingModifications][end][:drivingCourseModified])

        #fill up the rest of the driving course with information from the original course
        drivingCourseNew[end][:F_T]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:F_T]
        drivingCourseNew[end][:R_traction]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:R_traction]
        drivingCourseNew[end][:R_wagons]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:R_wagons]
        drivingCourseNew[end][:R_train]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:R_train]
        drivingCourseNew[end][:R_path]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:R_path]
        drivingCourseNew[end][:F_R]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:F_R]
        drivingCourseNew[end][:a]=drivingCourseOriginal[lastIdOfSelectedCsOriginal][:a]

        endOfModificationId=drivingCourseNew[end][:i]                 # is needed for updating the other modified driving courses
        difference=endOfModificationId-lastIdOfSelectedCsOriginal

        i=lastIdOfSelectedCsOriginal+1
        while i <= length(drivingCourseOriginal)
            push!(drivingCourseNew, copy(drivingCourseOriginal[i]))
            drivingCourseNew[end][:i]=length(drivingCourseNew)
            drivingCourseNew[end][:t]=drivingCourseNew[end-1][:t]+drivingCourseNew[end][:Δt]
            drivingCourseNew[end][:E]=drivingCourseNew[end-1][:E]+drivingCourseNew[end][:ΔE]
            drivingCourseNew[end][:W]=drivingCourseNew[end-1][:W]+drivingCourseNew[end][:ΔW]
            i=i+1
        end # while

        # replace the original driving course and CS with the new modified ones
        drivingCourseOriginal=drivingCourseNew
        CSsOrig[csIdMax]=copyCharacteristicSection(movingSectionOriginal[:energySavingModifications][end][:csModified])
        # 01/09 old with copy: CSsOrig[csIdMax]=copy(movingSectionOriginal[:energySavingModifications][end][:csModified])
        movingSectionOriginal[:t]=drivingCourseOriginal[end][:t]            # total running time (in s)
        movingSectionOriginal[:E]=drivingCourseOriginal[end][:E]            # total energy consumption (in Ws)

        # update all the data point references in the behaviour sections of the following characteristic sections and the other modified characteristic sections
        if difference!= 0
            # update the data point references in the behaviour sections of the following characteristic sections
            allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :braking, :standstill]
            for csId in csIdMax+1:length(CSsOrig)
                for bs in 1: length(allBs)
                    if haskey(CSsOrig[csId][:behaviorSections], allBs[bs])
                        for point in 1:length(CSsOrig[csId][:behaviorSections][allBs[bs]][:dataPoints])
                            CSsOrig[csId][:behaviorSections][allBs[bs]][:dataPoints][point] = CSsOrig[csId][:behaviorSections][allBs[bs]][:dataPoints][point]+difference
                        end
                    end #if
                end #for
            end #for

            # update the data points in the following modified charateristic sections and the following points in the driving course
            energySavingModificationsWithCoasting = updateEnergySavingModifications!(energySavingModificationsWithCoasting, csIdMax, drivingCourseNew, endOfModificationId, lastIdOfSelectedCsOriginal)
            energySavingModificationsWithMaximumSpeed = updateEnergySavingModifications!(energySavingModificationsWithMaximumSpeed, csIdMax, drivingCourseNew, endOfModificationId, lastIdOfSelectedCsOriginal)
            energySavingModificationsWithCombination = updateEnergySavingModifications!(energySavingModificationsWithCombination, csIdMax, drivingCourseNew, endOfModificationId, lastIdOfSelectedCsOriginal)
        end # if difference

        # modify new CS for the considered methods
        # method 1: increase coasting
        if doMethod1==true
            modificationType = "increasing coasting"
            energySavingModification = modifyCs(movingSectionOriginal, drivingCourseOriginal, csIdMax, modificationType, settings, train)
            energySavingModificationsWithCoasting[csIdMax]=energySavingModification
        end #if if doMethod1

        # method 2: accelerate to a lower v_peak
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

## copy the different sections the whole path can be devided in
function copyMovingSection(original::Dict)
    copiedCSs = Vector{Dict}()
    for csId in 1:length(original[:characteristicSections])
        push!(copiedCSs, copyCharacteristicSection(original[:characteristicSections][csId]))
        # 01/07 old without copy: push!(copiedCSs, copyCharacteristicSection(original[:characteristicSections][csId]))
    end #for

    copiedMS = Dict(:id => original[:id],                                   # identifier
                :length => original[:length],                               # total length (in m)
                :s_entry => original[:s_entry],                             # first position (in m)
                :s_exit => original[:s_exit],                               # last position (in m)
                :t => original[:t],                                         # total running time (in s)
                :E => original[:E],                                         # total energy consumption (in Ws)
                :characteristicSections => copiedCSs)                         # list of containing characteristic sections

    if haskey(original, :energySavingModifications)         # list of containing all the used energy saving modifications
        copiedModifications = Dict[]
        for modId in 1:length(original[:energySavingModifications])
            push!(copiedModifications, copyEnergySavingModification(original[:energySavingModifications][modId]))
        end #for
        merge!(copiedMS, Dict(:energySavingModifications => copiedModifications))
    end

    if haskey(original, :t_recovery)         # total recovery time for energy-saving modifications (in s)
        merge!(copiedMS, Dict(:t_recovery => original[:t_recovery]))
    end

    if haskey(original, :t_recoveryAvailable)         # still available recovery time for energy-saving modifications (in s)
        merge!(copiedMS, Dict(:t_recoveryAvailable => original[:t_recoveryAvailable]))
    end
    return copiedMS
end #function copyMovingSection

function copyCharacteristicSection(originalCS::Dict)
    allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :braking, :standstill]
    copiedBSs = Dict()
    for bs in 1: length(allBs)
        if haskey(originalCS[:behaviorSections], allBs[bs])
            merge!(copiedBSs, Dict(allBs[bs] => originalCS[:behaviorSections][allBs[bs]]))
        end #if
    end #for

    copiedCS=Dict(:id => originalCS[:id],               # identifier
                 :s_entry => originalCS[:s_entry],      # first position (in m)
                 :s_exit => originalCS[:s_exit],        # last position  (in m)
                 :length => originalCS[:length],        # total length  (in m)
                 :r_path => originalCS[:r_path],        # path resistance (in ‰)
                #      :behaviorSections => copy(originalCS[:behaviorSections]),  # list of containing behavior sections
                 :behaviorSections => copiedBSs,  # list of containing behavior sections
                 :t => originalCS[:t],                  # total running time (in s)
                 :E => originalCS[:E],                  # total energy consumption (in Ws)
                 :v_limit => originalCS[:v_limit],      # speed limit (in m/s)
                 :v_peak => originalCS[:v_peak],        # maximum reachable speed (in m/s)
                 :v_entry => originalCS[:v_entry],      # maximum entry speed (in m/s)
                 :v_exit => originalCS[:v_exit],        # maximum exit speed (in m/s)
                 :pointsOfInterest => originalCS[:pointsOfInterest])     # points of interest for which data points should be calculated

        return copiedCS
end # CharacteristicSection

# smallest section of the path is the behavior section. It relates to the containing data points via their identifier.
function copyBehaviorSection(original::Dict)
    bsDataPoints=[]
    for i in 1:length(original[:dataPoints])
        push!(bsDataPoints, original[:dataPoints][i])
    end
    copiedBS = Dict(#:type => behavior,                 # type of behavior section: breakFree, clearing, acceleration, cruising, diminishing, coasting, braking or standstill
                :type => original[:type],           # type of behavior section: "breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "braking" or "standstill"
                :length => original[:length],       # total length  (in m)
                :s_entry => original[:s_entry],     # first position (in m)
                :s_exit => original[:s_exit],       # last position  (in m)
                :t => original[:t],                 # total running time (in s)
                :E => original[:E],                 # total energy consumption (in Ws)
                :v_entry => original[:v_entry],     # entry speed (in m/s)
                :v_exit => original[:v_exit],       # exit speed (in m/s)
                :dataPoints => bsDataPoints)        # list of identifiers of the containing data points
    return copiedBS
end #function copyBehaviorSection

## for the energy saving operation mode it is nesserary to compare different energy saving modifications. These are part of the moving section.
function createEnergySavingModification()
    energySavingModification = Dict(:csId => 0,                     # identifier of the characteristic section
                                    :type => "",                    # type of energy saving modification: "increasing coasting" "decreasing maximum velocity" or "combination of decreasing maximum velocity and coasting"
                                    :ΔE => 0.0,                     # saved energy (in Ws)
                                    :Δt => 0.0,                     # time loss (in s)
                                    :ratio => 0.0,                  # ratio of ΔE and Δt (in Ws/s)
                                    :csModified => Dict(),          # the modified characteristic section
                                    :drivingCourseModified => [])   # drivingCourse for the modified characteristic section
end #createEnergySavingModification

function updateEnergySavingModifications!(energySavingModifications::Vector{Dict}, csIdMax::Integer, drivingCourseNew::Vector{Dict}, endOfModificationId::Integer, lastIdOfSelectedCsOriginal::Integer)
    allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :braking, :standstill]
    difference = endOfModificationId-lastIdOfSelectedCsOriginal
    for modNr in csIdMax+1:length(energySavingModifications)
        if energySavingModifications[modNr][:ratio]>0
            BSs = energySavingModifications[modNr][:csModified][:behaviorSections]
            # update the behavior sections of the modified charateristic section
            for bs in 1: length(allBs)
                if haskey(BSs, allBs[bs])
                    for point in 1:length(BSs[allBs[bs]][:dataPoints])
                        BSs[allBs[bs]][:dataPoints][point] = BSs[allBs[bs]][:dataPoints][point] + difference
                    end
                end #if
            end #for

            # correct the points of previous CS in the modified driving course. Copy the new driving course till the beginning of the current CS and change total values of the current modified CS data points accordingly
            drivingCourseModifiedNew = copy(drivingCourseNew[1:endOfModificationId])

            i=lastIdOfSelectedCsOriginal+1
            while i <= length(energySavingModifications[modNr][:drivingCourseModified])
                push!(drivingCourseModifiedNew, copy(energySavingModifications[modNr][:drivingCourseModified][i]))

                drivingCourseModifiedNew[end][:i]=length(drivingCourseModifiedNew)
                drivingCourseModifiedNew[end][:t]=drivingCourseModifiedNew[end-1][:t]+drivingCourseModifiedNew[end][:Δt]
                drivingCourseModifiedNew[end][:E]=drivingCourseModifiedNew[end-1][:E]+drivingCourseModifiedNew[end][:ΔE]
                drivingCourseModifiedNew[end][:W]=drivingCourseModifiedNew[end-1][:W]+drivingCourseModifiedNew[end][:ΔW]
                i=i+1
            end # while

            energySavingModifications[modNr][:drivingCourseModified] = drivingCourseModifiedNew
        end #if
    end #for

    return energySavingModifications
end #function updateEnergySavingModifications!

function copyEnergySavingModification(modificaionOriginal::Dict)
    modificaionCopy = Dict(:csId => modificaionOriginal[:csId],                     # identifier of the characteristic section
                            :type => modificaionOriginal[:type],                    # type of energy saving modification: "increasing coasting" "decreasing maximum velocity" or "combination of decreasing maximum velocity and coasting"
                            :ΔE => modificaionOriginal[:ΔE],                     # saved energy (in Ws)
                            :Δt => modificaionOriginal[:Δt],                     # time loss (in s)
                            :ratio => modificaionOriginal[:ratio],                  # ratio of ΔE and Δt (in Ws/s)
                            :csModified => copyCharacteristicSection(modificaionOriginal[:]),          # the modified characteristic section
                            :drivingCourseModified => copy(modificaionOriginal[:drivingCourseModified]))   # drivingCourse for the modified characteristic section

    return modificaionCopy
end # copyEnergySavingModification

function modifyCs(movingSectionOriginal::Dict, drivingCourseOriginal::Vector{Dict}, csId::Integer, modificationType::String, settings::Dict, train::Dict)
 # TODO: refactor and sort this function
    CSsOrig::Vector{Dict} = movingSectionOriginal[:characteristicSections]

    if modificationType == "increasing coasting"
        # method 1: increase coasting
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=increaseCoastingSection(CSsOrig[csId], drivingCourseOriginal, settings, train, CSsOrig, movingSectionOriginal[:t_recoveryAvailable])
    elseif modificationType == "decreasing maximum velocity"
        # method 2: accelerate to a lower v_peak
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=decreaseMaximumVelocity(CSsOrig[csId], drivingCourseOriginal, settings, train, CSsOrig, movingSectionOriginal[:t_recoveryAvailable])

    elseif modificationType == "combination of energy saving methods"
        # calculate the combination of the previous methods
        (characteristicSectionModified, drivingCourseModifiedUntilEndOfModifiedCS, new)=combineEnergySavingMethods(CSsOrig[csId], drivingCourseOriginal, settings, train, CSsOrig, movingSectionOriginal[:t_recoveryAvailable])
    else
        return createEnergySavingModification()
    end

    #energySavingModification = createEnergySavingModification()
    if new
        energySavingModification = Dict(:csId => csId,                                  # identifier of the characteristic section
                                        :type => modificationType,                      # type of energy saving modification: "increasing coasting" "decreasing maximum velocity" or "combination of decreasing maximum velocity and coasting"
                                        :csModified => characteristicSectionModified,   # the modified characteristic section
                                        :drivingCourseModified => drivingCourseModifiedUntilEndOfModifiedCS)          # drivingCourse for the modified characteristic section

        merge!(energySavingModification, Dict(:ΔE => CSsOrig[csId][:E] - energySavingModification[:csModified][:E]))                     # saved energy (in Ws)
        merge!(energySavingModification, Dict(:Δt => energySavingModification[:csModified][:t] - CSsOrig[csId][:t]))                     # time loss (in s)


        if energySavingModification[:Δt] <= movingSectionOriginal[:t_recoveryAvailable] && energySavingModification[:ΔE] >= 0.0
            #*** TODO: check why "sign" is needed here
        #    if modificationType == "combination of energy saving methods"
            ratio=sign(energySavingModification[:Δt])*energySavingModification[:ΔE]/energySavingModification[:Δt]        # ratio of ΔE and Δt (in Ws/s)

        #    else
        #            ratio = energySavingModification[:ΔE] / energySavingModification[:Δt]          # ratio of ΔE and Δt (in Ws/s)
        #    end
            # ***
        elseif energySavingModification[:Δt] == 0.0
            ratio = energySavingModification[:ΔE]/0.000000001
        else # Δt is to high or ΔE < 0.0 Ws
            ratio = 0.0
        end
        merge!(energySavingModification, Dict(:ratio => ratio))                  # ratio of ΔE and Δt (in Ws/s)
        return energySavingModification
    else
        return createEnergySavingModification()
    end
end #function modifyCs

function findBestModification(energySavingModifications::Vector{Dict}, ratioMax::AbstractFloat, csIdMax::Integer, typeMax::String, t_recoveryAvailable::AbstractFloat)
    for modNr in 1:length(energySavingModifications)
        if energySavingModifications[modNr][:ratio] > ratioMax
            if energySavingModifications[modNr][:Δt] <= t_recoveryAvailable
                ratioMax = energySavingModifications[modNr][:ratio]
                csIdMax = energySavingModifications[modNr][:csId]
                typeMax = energySavingModifications[modNr][:type]
            else # Δt is to high
                energySavingModifications[modNr][:ratio]=0.0
            end #if
        end #if
    end #for
    return (energySavingModifications, ratioMax, csIdMax, typeMax)
end #function findBestModification


## functions for calculating different energy saving methods that are used in calculateMinimumEnergyConsumption

function calculateRecoveryTime(s_MS::Real, t_MS::AbstractFloat, train::Dict)
    # function for calculating the recovery time that can be used for energy saving
    # MS: Moving Section
    # 01/05 old without enum: if train[:type]=="motor coach train"
    if train[:type] == motorCoachTrain::trainType
        if s_MS<= 30000
            c_s=0.0
        else s_MS> 30000
            c_s=0.0006
        end # if s_MS

        if train[:v_limit]<=140/3.6     # unit is m/s
            c_t=0.03
        elseif train[:v_limit]<=160/3.6 # unit is m/s
            c_t=0.04
        elseif train[:v_limit]<=200/3.6 # unit is m/s
            c_t=0.05
        elseif train[:v_limit]<=250/3.6 # unit is m/s
            c_t=0.06
        else # train[:v_limit]>120/3.6  # unit is m/s
            c_t=0.07
        end # if train[:v_limit]

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    # 01/05 old without enum: elseif train[:type]=="freight" && train[:v_limit]<=120/3.6   # unit is m/s
    elseif train[:type] == freight::trainType && train[:v_limit] <= 120/3.6   # unit is m/s
        t_recovery1=s_MS*0.0006 +t_MS*0.03
        t_recovery2=s_MS*0.0018 +t_MS*0.0
        t_recovery3=s_MS*0.0 +t_MS*0.04
        t_recovery=max(t_recovery1, t_recovery2, t_recovery3)

        return t_recovery
    # 01/05 old without enum: else # train[:trainType]=="passenger" || (train[:trainType]=="freight" && train[:v_limit]>120/3.6) # unit is m/s
    else # train[:type] == passenger::trainType || (train[:type] == freight::trainType && train[:v_limit]>120/3.6) # unit is m/s
        if s_MS<= 30000
            c_s=0.0
        else s_MS> 30000
            c_s=0.0009
        end # if s_MS
        if train[:v_limit]<=140/3.6         # unit is m/s
            if train[:m_train]<=300000      # unit is kg
                c_t=0.03
            elseif train[:m_train]<=500000  # unit is kg
                c_t=0.04
            elseif train[:m_train]<=700000  # unit is kg
                c_t=0.04
            else # train[:m_train]>700000   # unit is kg
                c_t=0.05
            end # if train[:m_train]
        elseif train[:v_limit]<=160/3.6     # unit is m/s
            if train[:m_train]<=300000      # unit is kg
                c_t=0.03
            elseif train[:m_train]<=500000  # unit is kg
                c_t=0.04
            else # train[:m_train]>500000   # unit is kg
                c_t=0.0
            end # if train[:m_train]
        elseif train[:v_limit]<=200/3.6     # unit is m/s
            if train[:m_train]<=300000      # unit is kg
                c_t=0.04
            elseif train[:m_train]<=500000  # unit is kg
                c_t=0.05
            else # train[:m_train]>500000   # unit is kg
                c_t=0.06
            end # if train[:m_train]
        else # train[:v_limit]>200/3.6      # unit is m/s
            if train[:m_train]<=300000      # unit is kg
                c_t=0.05
            elseif train[:m_train]<=500000  # unit is kg
                c_t=0.06
            else # train[:m_train]>500000   # unit is kg
                c_t=0.07
            end # if train[:m_train]
        end # if train[:v_limit]

        c_tMin=s_MS/t_MS*0.0012
        c_t=max(c_t, c_tMin)

        t_recovery=s_MS*c_s+t_MS*c_t
        return t_recovery
    end # if train[:type]
end #function calculateRecoveryTime

# TODO: a refactoring caused worse drivingsCourses. see the commented function below
function increaseCoastingSection(csOriginal::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, allCSs::Vector{Dict}, t_recoveryAvailable::AbstractFloat)
    BSsOriginal = csOriginal[:behaviorSections]
    if (haskey(BSsOriginal, :cruising) || (haskey(BSsOriginal, :diminishing) && get(BSsOriginal, :diminishing, Dict(:dataPoints =>[0]))[:dataPoints][1] > get(BSsOriginal, :acceleration, Dict(:dataPoints =>[0]))[:dataPoints][1])) && haskey(BSsOriginal, :braking)
        # check if cruising or diminishing should be reduced for coasting
        if haskey(BSsOriginal, :cruising) && haskey(BSsOriginal, :diminishing)
            if BSsOriginal[:cruising][:dataPoints][1] > BSsOriginal[:diminishing][:dataPoints][1]
                reduceCruising=true
                reduceDiminishing=false
            else
                reduceDiminishing=true
                reduceCruising=false
            end
        elseif haskey(BSsOriginal, :cruising)
            reduceCruising=true
            reduceDiminishing=false
        elseif haskey(BSsOriginal, :diminishing)
            reduceDiminishing=true
            reduceCruising=false
        else
            error("in increaseCoastingSection") #TODO
        end
#=
        # copy csOriginal to csModifiedInitial
        csModifiedInitial = copyCharacteristicSection(csOriginal)
        BSsModified = csModifiedInitial[:behaviorSections]

        # delete bahavior sections that will be recalculated except breakFree, clearing, acceleration, diminishing
        # and rest total running time and energy consumption
        if haskey(BSsModified, :coasting)
            csModifiedInitial[:E] = csModifiedInitial[:E] - BSsModified[:coasting][:E]
            csModifiedInitial[:t] = csModifiedInitial[:t] - BSsModified[:coasting][:t]
            delete!(BSsModified, :coasting)
        end
        if haskey(BSsModified, :braking)
            csModifiedInitial[:E] = csModifiedInitial[:E] - BSsModified[:braking][:E]
            csModifiedInitial[:t] = csModifiedInitial[:t] - BSsModified[:braking][:t]
            delete!(BSsModified, :braking)
        end
        if haskey(BSsModified, :standstill)
            csModifiedInitial[:E] = csModifiedInitial[:E] - BSsModified[:standstill][:E]
            csModifiedInitial[:t] = csModifiedInitial[:t] - BSsModified[:standstill][:t]
            delete!(BSsModified, :standstill)
        end


        if reduceCruising
            cruisingReduction = settings[:stepSize]
                # 01/07 test for a better calculation time: cruisingReduction = settings[:stepSize]*100

            # remove old cruising section
            csModifiedInitial[:E] = csModifiedInitial[:E] - BSsModified[:cruising][:E]
            csModifiedInitial[:t] = csModifiedInitial[:t] - BSsModified[:cruising][:t]
            delete!(BSsModified, :cruising)

            # determine the starting point of saving energy (needed for copying the characteristic section's drivingCourse)
            energySavingStartId=get(BSsOriginal, :cruising, Dict(:dataPoints=>[0]))[:dataPoints][1]
            if energySavingStartId==0
                error("ERROR at creating a new driving course for energy saving with coasting !")
            end

            while cruisingReduction>=settings[:stepSize]/10^approximationLevel
            #while cruisingReduction>=settings[:stepSize]/100
                while cruisingReduction>=settings[:stepSize]/10^approximationLevel # will be done once and then depending on approximationLevel repeated with smaller cruisingReduction unless !(drivingCourseModified[end][:v]<=csModified[:v_exit] && drivingCourseModified[end][:s]<csModified[:s_exit]) -> see below at the end of the while loop
                    # copy the characteristic section for being modified
                    csModified = copyCharacteristicSection(csModifiedInitial)

                    # copy the driving course till the beginning of energy saving
                    drivingCourseModified = copy(drivingCourse[1:energySavingStartId])         # List of data points till the start of energy saving

                    # calculating the new length of the cruising section
                    if settings[:stepVariable]=="s in m"                                                           # distance step method
                        s_cruising = BSsOriginal[:cruising][:length] - cruisingReduction
                    elseif settings[:stepVariable]=="t in s"                                                       # time step method
                        # 09/20 old: doesn't work for non constant cruising -> TODO: should work now
                            # t_cruising=BSsOriginal[:cruising][:t]-cruisingReduction
                            # s_cruising=t_cruising*drivingCourseModified[end][:v]
                        distanceReduction = drivingCourse[BSsOriginal[:cruising][:dataPoints][end]][:v] * cruisingReduction
                        s_cruising = BSsOriginal[:cruising][:length]-distanceReduction

                    elseif settings[:stepVariable]=="v in m/s"                                                       # velocity step method
                        s_cruising=BSsOriginal[:cruising][:length]-cruisingReduction*10 # TODO: or better: *100 ?
                    end #if
                    s_cruising=max(0.0, s_cruising)

                    # calculate the new and now shorter cruising section
                    if s_cruising>0.0
                        (csModified, drivingCourseModified)=addCruisingSection!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
                    end

                    # calculate the coasting phase until the point the train needs to brake
                    (csModified, drivingCourseModified)=addCoastingSection!(csModified, drivingCourseModified, settings, train, allCSs)

                    if drivingCourseModified[end][:v] < csModified[:v_exit] || drivingCourseModified[end][:s] > csModified[:s_exit]
                        # the train reaches v_exit before reaching s_exit. The cruising and coasting sections have to be calculated again with a larger cruising section (so with a smaller reduction of the cruising section)
                        cruisingReduction=cruisingReduction/10
                    else
                        break
                    end
                end # while cruisingReduction

                # calculate the moving phase between coasting and the end of the CS
                if drivingCourseModified[end][:v] > csModified[:v_exit]
                    #(csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings[:massModel], train, allCSs)
                    (csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings, train, allCSs)
                end

                if t_recoveryAvailable < csModified[:t]-csOriginal[:t] || drivingCourseModified[end][:v] != csModified[:v_exit] || drivingCourseModified[end][:s] != csModified[:s_exit] # time loss is to high and the CS has to be calculated again with larger cruising section (so with a smaller reduction of the cruising section) or v_exit or s_exit are not reached excatly
                    cruisingReduction=cruisingReduction/10
                else
                    return (csModified, drivingCourseModified, true)
                end
            end #while


        elseif reduceDiminishing
            # TODO: At the moment diminishing is reduced similar to the acceleration in decreaseMaximumVelocity. To reduce code the methods for reducing cruising phase and reducing the diminishing phase can be combined in some parts.
            csModified = csModifiedInitial
            diminishingSection = BSsModified[:diminishing]

            # remove the last diminishing waypoint
            t_diff = drivingCourse[diminishingSection[:dataPoints][end]][:t] - drivingCourse[diminishingSection[:dataPoints][end-1]][:t]
            E_diff = drivingCourse[diminishingSection[:dataPoints][end]][:E] - drivingCourse[diminishingSection[:dataPoints][end-1]][:E]
            pop!(diminishingSection[:dataPoints])

            diminishingSection[:v_exit] = drivingCourse[diminishingSection[:dataPoints][end]][:v]                                   # exit speed (in m/s)
            diminishingSection[:s_exit] = drivingCourse[diminishingSection[:dataPoints][end]][:s]                                   # last position (in m)
            diminishingSection[:length] = diminishingSection[:s_exit] - diminishingSection[:s_entry]                                # total length  (in m)
            diminishingSection[:t] = diminishingSection[:t] - t_diff                                                                # total running time (in s)
            diminishingSection[:E] = diminishingSection[:E] - E_diff                                                                # total energy consumption (in Ws)

        #    merge!(BSsModified, Dict(:diminishing => diminishingSection))
            csModified[:E] = csModified[:E] - t_diff
            csModified[:t] = csModified[:t] - E_diff

            energySavingStartId = diminishingSection[:dataPoints][end]

           if length(diminishingSection[:dataPoints]) == 2          # The diminishing section was only one step. This step is removed and so the complette diminishing section.
                delete!(BSsModified, :diminishing)
            end

#_____________
=#

        if reduceCruising
            cruisingReduction = settings[:stepSize]
                # 01/07 test for a better calculation time: cruisingReduction = settings[:stepSize]*100
            while cruisingReduction>=settings[:stepSize]/10^approximationLevel
            #while cruisingReduction>=settings[:stepSize]/100
                while cruisingReduction>=settings[:stepSize]/10^approximationLevel # will be done once and then depending on approximationLevel repeated with smaller cruisingReduction unless !(drivingCourseModified[end][:v]<=csModified[:v_exit] && drivingCourseModified[end][:s]<csModified[:s_exit]) -> see below at the end of the while loop

                    # create a copy for the characteristic sections drivingCourse
                    energySavingStartId=get(BSsOriginal, :cruising, Dict(:dataPoints=>[0]))[:dataPoints][1]
                    if energySavingStartId==0
                        error("ERROR at creating a new driving course for energy saving with coasting !")
                    end

                    # copy the driving course till the beginning of energy saving
                    drivingCourseModified = copy(drivingCourse[1:energySavingStartId])  # List of data points till the start of energy saving

                    # calculating the new length of the cruising section
                    if settings[:stepVariable]=="s in m"                                                           # distance step method
                        s_cruising = BSsOriginal[:cruising][:length] - cruisingReduction
                    elseif settings[:stepVariable]=="t in s"                                                       # time step method
                        # 09/20 old: doesn't work for non constant cruising -> TODO: should work now
                            # t_cruising=BSsOriginal[:cruising][:t]-cruisingReduction
                            # s_cruising=t_cruising*drivingCourseModified[end][:v]
                        distanceReduction = drivingCourse[BSsOriginal[:cruising][:dataPoints][end]][:v]*cruisingReduction
                        s_cruising = BSsOriginal[:cruising][:length]-distanceReduction

                    elseif settings[:stepVariable]=="v in m/s"                                                       # velocity step method
                        s_cruising=BSsOriginal[:cruising][:length]-cruisingReduction*10 # TODO: or better: *100 ?
                    end #if
                    s_cruising=max(0.0, s_cruising)

                    # copy csOriginal to csModified
                    # 12/28 old: csModified=CharacteristicSection(csOriginal[:id], csOriginal[:length], csOriginal[:s_entry], csOriginal[:s_exit], 0.0, 0.0, csOriginal[:v_limit], csOriginal[:v_peak], csOriginal[:v_entry], csOriginal[:v_exit], csOriginal[:r_path], Dict{Symbol, Dict}())
                    #TODO after removing the mutable structs: Is it possible to just "copy"? with some changes
                    csModified=Dict(:id => csOriginal[:id],             # identifier
                                    :s_entry => csOriginal[:s_entry],   # first position (in m)
                                    :s_exit => csOriginal[:s_exit],     # last position  (in m)
                                    :length => csOriginal[:length],     # total length  (in m)
                                    :r_path => csOriginal[:r_path],     # path resistance (in ‰)
                                    :behaviorSections => Dict(),        # empty list of containing behavior sections
                                    :t => 0.0,                          # total running time (in s)
                                    :E => 0.0,                          # total energy consumption (in Ws)
                                    :v_limit => csOriginal[:v_limit],   # speed limit (in m/s)
                                    :v_peak => csOriginal[:v_peak],     # maximum reachable speed (in m/s)
                                    :v_entry => csOriginal[:v_entry],   # maximum entry speed (in m/s)
                                    :v_exit => csOriginal[:v_exit],     # maximum exit speed (in m/s)
                                    :pointsOfInterest => csOriginal[:pointsOfInterest])     # points of interest for which data points should be calculated

                    BSsModified = csModified[:behaviorSections]
                    if haskey(BSsOriginal, :breakFree)
                        breakFreeSection=copyBehaviorSection(BSsOriginal[:breakFree])
                        merge!(BSsModified, Dict(:breakFree=>breakFreeSection))
                        csModified[:E] = csModified[:E] + BSsModified[:breakFree][:E]
                        csModified[:t] = csModified[:t] + BSsModified[:breakFree][:t]
                    end
                    if haskey(BSsOriginal, :clearing)            # this section is needed before acceleration if the train wants to accelerate to a speed higher than the limit in a previous CS where parts of the train are still located
                        clearingSection=copyBehaviorSection(BSsOriginal[:clearing])
                        merge!(BSsModified, Dict(:clearing=>clearingSection))
                        csModified[:E] = csModified[:E] + BSsModified[:clearing][:E]
                        csModified[:t] = csModified[:t] + BSsModified[:clearing][:t]
                    end
                    if haskey(BSsOriginal, :acceleration)
                        accelerationSection=copyBehaviorSection(BSsOriginal[:acceleration])
                        merge!(BSsModified, Dict(:acceleration=>accelerationSection))
                        csModified[:E] = csModified[:E] + BSsModified[:acceleration][:E]
                        csModified[:t] = csModified[:t] + BSsModified[:acceleration][:t]
                    end
                    if haskey(BSsOriginal, :diminishing)
                        diminishingSection=copyBehaviorSection(BSsOriginal[:diminishing])
                        merge!(BSsModified, Dict(:diminishing=>diminishingSection))
                        csModified[:E] = csModified[:E] + BSsModified[:diminishing][:E]
                        csModified[:t] = csModified[:t] + BSsModified[:diminishing][:t]
                    end


                    # calculate the new and now shorter cruising section
                    if s_cruising>0.0
                        (csModified, drivingCourseModified)=addCruisingSection!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
                    end

                    # calculate the coasting phase until the point the train needs to brake
                    (csModified, drivingCourseModified)=addCoastingSection!(csModified, drivingCourseModified, settings, train, allCSs)

                    if drivingCourseModified[end][:v] < csModified[:v_exit] || drivingCourseModified[end][:s] > csModified[:s_exit]
                        # the train reaches v_exit before reaching s_exit. The cruising and coasting sections have to be calculated again with a larger cruising section (so with a smaller reduction of the cruising section)
                        cruisingReduction=cruisingReduction/10
                    else
                        break
                    end
                end # while cruisingReduction

                # calculate the moving phase between coasting and the end of the CS
                if drivingCourseModified[end][:v] > csModified[:v_exit]
                    #(csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings[:massModel], train, allCSs)
                    (csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings, train, allCSs)
                end

                if t_recoveryAvailable < csModified[:t]-csOriginal[:t] || drivingCourseModified[end][:v] != csModified[:v_exit] || drivingCourseModified[end][:s] != csModified[:s_exit] # time loss is to high and the CS has to be calculated again with larger cruising section (so with a smaller reduction of the cruising section) or v_exit or s_exit are not reached excatly
                    cruisingReduction=cruisingReduction/10
                else
                    return (csModified, drivingCourseModified, true)
                end
            end #while


        elseif reduceDiminishing
            # TODO: At the moment diminishing is reduced like the acceleration in decreaseMaximumVelocity. To reduce code the methods for reducing cruising phase and reducing the diminishing phase can be combined in some parts.

            # copy csOriginal to csModified
            # 12/28 old: csModified=CharacteristicSection(csOriginal[:id], csOriginal[:length], csOriginal[:s_entry], csOriginal[:s_exit], 0.0, 0.0, csOriginal[:v_limit], csOriginal[:v_peak], csOriginal[:v_entry], csOriginal[:v_exit], csOriginal[:r_path], Dict{Symbol, Dict}())
            csModified=Dict(:id => csOriginal[:id],             # identifier
                            :s_entry => csOriginal[:s_entry],   # first position (in m)
                            :s_exit => csOriginal[:s_exit],     # last position  (in m)
                            :length => csOriginal[:length],     # total length  (in m)
                            :r_path => csOriginal[:r_path],     # path resistance (in ‰)
                            :behaviorSections => Dict(),        # empty list of containing behavior sections
                            :t => 0.0,                          # total running time (in s)
                            :E => 0.0,                          # total energy consumption (in Ws)
                            :v_limit => csOriginal[:v_limit],   # speed limit (in m/s)
                            :v_peak => csOriginal[:v_peak],     # maximum reachable speed (in m/s)
                            :v_entry => csOriginal[:v_entry],   # maximum entry speed (in m/s)
                            :v_exit => csOriginal[:v_exit],     # maximum exit speed (in m/s)
                            :pointsOfInterest => csOriginal[:pointsOfInterest])     # points of interest for which data points should be calculated

            BSsModified = csModified[:behaviorSections]
            if haskey(BSsOriginal, :breakFree)
                breakFreeSection=copyBehaviorSection(BSsOriginal[:breakFree])
                merge!(BSsModified, Dict(:breakFree=>breakFreeSection))
                csModified[:E] = csModified[:E] + BSsModified[:breakFree][:E]
                csModified[:t] = csModified[:t] + BSsModified[:breakFree][:t]
            end
            if haskey(BSsOriginal, :clearing)            # this section is needed before acceleration if the train wants to accelerate to a speed higher than the limit in a previous CS where parts of the train are still located
                clearingSection=copyBehaviorSection(BSsOriginal[:clearing])
                merge!(BSsModified, Dict(:clearing=>clearingSection))
                csModified[:E] = csModified[:E] + BSsModified[:clearing][:E]
                csModified[:t] = csModified[:t] + BSsModified[:clearing][:t]
            end
            if haskey(BSsOriginal, :acceleration)
                accelerationSection=copyBehaviorSection(BSsOriginal[:acceleration])
                merge!(BSsModified, Dict(:acceleration=>accelerationSection))
                csModified[:E] = csModified[:E] + BSsModified[:acceleration][:E]
                csModified[:t] = csModified[:t] + BSsModified[:acceleration][:t]
            end
            if haskey(BSsOriginal, :cruising)
                cruisingSection=copyBehaviorSection(BSsOriginal[:cruising])
                merge!(BSsModified, Dict(:cruising=>cruisingSection))
                csModified[:E] = csModified[:E] + BSsModified[:cruising][:E]
                csModified[:t] = csModified[:t] + BSsModified[:cruising][:t]
            end

            diminishingSection=copyBehaviorSection(BSsOriginal[:diminishing])
            if length(diminishingSection[:dataPoints]) > 2
                # remove the last diminishing waypoint
                pop!(diminishingSection[:dataPoints])

                diminishingSection[:v_exit]=drivingCourse[diminishingSection[:dataPoints][end]][:v]                               # exit speed (in m/s)
                diminishingSection[:s_exit]=drivingCourse[diminishingSection[:dataPoints][end]][:s]                                # last position (in m)
                diminishingSection[:length]=diminishingSection[:s_exit]-diminishingSection[:s_entry]           # total length  (in m)
                diminishingSection[:t]=drivingCourse[diminishingSection[:dataPoints][end]][:t]-drivingCourse[diminishingSection[:dataPoints][1]][:t]       # total running time (in s)
                diminishingSection[:E]=drivingCourse[diminishingSection[:dataPoints][end]][:E]-drivingCourse[diminishingSection[:dataPoints][1]][:E]       # total energy consumption (in Ws)

                merge!(BSsModified, Dict(:diminishing => diminishingSection))
                csModified[:E] = csModified[:E] + BSsModified[:diminishing][:E]
                csModified[:t] = csModified[:t] + BSsModified[:diminishing][:t]

                energySavingStartId=diminishingSection[:dataPoints][end]
            else
                # The diminishing section is only one step. This step is removed and if there is a clearing section it will be combined with the new cruising section.
                energySavingStartId=get(BSsOriginal, :clearing, get(BSsOriginal, :diminishing, Dict(:dataPoints =>[0])))[:dataPoints][1]
            end

            # copy the driving course till the beginning of energy saving
            drivingCourseModified = copy(drivingCourse[1:energySavingStartId])  # List of data points till the start of energy saving

            # calculate the coasting phase until the point the train needs to brake
            (csModified, drivingCourseModified)=addCoastingSection!(csModified, drivingCourseModified, settings, train, allCSs)

            # calculate the moving phase between coasting and the end of the CS
            if drivingCourseModified[end][:v] > csModified[:v_exit]
                (csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings, train, allCSs)
            end

            if t_recoveryAvailable >= csModified[:t] - csOriginal[:t]
                return (csModified, drivingCourseModified, true)
            else # time loss is to high. so there is no energy saving modification for this CS with the available recovery time
                # TODO: just return false or take smaller steps?

                # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
                return (csOriginal, drivingCourse, false)
            end
        end

        # there is no energy saving modification for this CS with the available recovery time
        # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
        return (csOriginal, drivingCourse, false)
    else
        # there is no energy saving modification for this CS because a cruising section AND a braking section are needed to be transformed into a coasting section
        # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
        return (csOriginal, drivingCourse, false)
    end
end # function increaseCoastingSection

# method 2 with shortening the acceleration by stepsize
function decreaseMaximumVelocity(csOriginal::Dict, drivingCourse, settings::Dict, train::Dict, allCSs::Vector{Dict}, t_recoveryAvailable::AbstractFloat)
    # TODO doesn't work that well alone. works better with combineEnergySavingMethods. why? does a while loop end to early or something like this?
    #function decreaseMaximumVelocity(csOriginal::CharacteristicSection, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, allCSs::Vector{CharacteristicSection}, t_recoveryAvailable::AbstractFloat)
    BSsOriginal = csOriginal[:behaviorSections]
    if haskey(BSsOriginal, :acceleration) && csOriginal[:v_peak] > csOriginal[:v_entry] && csOriginal[:v_peak] > csOriginal[:v_exit]
        accelerationSection = copyBehaviorSection(BSsOriginal[:acceleration])

        if drivingCourse[accelerationSection[:dataPoints][end]-1][:v] < csOriginal[:v_exit]
            # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
            return (csOriginal, drivingCourse, false)
            # TODO: or calculate a new acceleration phase with v_exit as v_peak? it will be very short, shorter than the step size.
        end

        # copy csOriginal to csModified
        # 12/28 old: csModified=CharacteristicSection(csOriginal[:id], csOriginal[:length], csOriginal[:s_entry], csOriginal[:s_exit], 0.0, 0.0, csOriginal[:v_limit], csOriginal[:v_peak], csOriginal[:v_entry], csOriginal[:v_exit], csOriginal[:r_path], Dict{Symbol, Dict}())
        csModified=Dict(:id => csOriginal[:id],             # identifier
                        :s_entry => csOriginal[:s_entry],   # first position (in m)
                        :s_exit => csOriginal[:s_exit],     # last position  (in m)
                        :length => csOriginal[:length],     # total length  (in m)
                        :r_path => csOriginal[:r_path],     # path resistance (in ‰)
                        :behaviorSections => Dict(),        # empty list of containing behavior sections
                        :t => 0.0,                          # total running time (in s)
                        :E => 0.0,                          # total energy consumption (in Ws)
                        :v_limit => csOriginal[:v_limit],   # speed limit (in m/s)
                        :v_peak => csOriginal[:v_peak],     # maximum reachable speed (in m/s)
                        :v_entry => csOriginal[:v_entry],   # maximum entry speed (in m/s)
                        :v_exit => csOriginal[:v_exit],     # maximum exit speed (in m/s)
                        :pointsOfInterest => csOriginal[:pointsOfInterest])     # points of interest for which data points should be calculated

        BSsModified = csModified[:behaviorSections]
        if haskey(BSsOriginal, :breakFree)
            breakFreeSection=copyBehaviorSection(BSsOriginal[:breakFree])
            merge!(BSsModified, Dict(:breakFree=>breakFreeSection))
            csModified[:E] = csModified[:E] + BSsModified[:breakFree][:E]
            csModified[:t] = csModified[:t] + BSsModified[:breakFree][:t]
        end
        if haskey(BSsOriginal, :diminishing) && BSsModified[:diminishing][:dataPoints][1] < BSsModified[:acceleration][:dataPoints][1]
            diminishingSection=copyBehaviorSection(BSsOriginal[:diminishing])
            merge!(BSsModified, Dict(:diminishing=>diminishingSection))
            csModified[:E] = csModified[:E] + BSsModified[:diminishing][:E]
            csModified[:t] = csModified[:t] + BSsModified[:diminishing][:t]
        end

        if length(accelerationSection[:dataPoints]) > 2
            if haskey(BSsOriginal, :clearing)
                clearingSection=copyBehaviorSection(BSsOriginal[:clearing])
                merge!(BSsModified, Dict(:clearing=>clearingSection))
                csModified[:E] = csModified[:E] + BSsModified[:clearing][:E]
                csModified[:t] = csModified[:t] + BSsModified[:clearing][:t]
            end

            # remove the last acceleration waypoint from the accelerationSection
            pop!(accelerationSection[:dataPoints])
            energySavingStartId = accelerationSection[:dataPoints][end]

            accelerationSection[:v_exit]=drivingCourse[energySavingStartId][:v]                               # exit speed (in m/s)
            accelerationSection[:s_exit]=drivingCourse[energySavingStartId][:s]                               # last position (in m)
            accelerationSection[:length]=accelerationSection[:s_exit]-accelerationSection[:s_entry]           # total length  (in m)
            accelerationSection[:t]=drivingCourse[energySavingStartId][:t]-drivingCourse[accelerationSection[:dataPoints][1]][:t]       # total running time (in s)
            accelerationSection[:E]=drivingCourse[energySavingStartId][:E]-drivingCourse[accelerationSection[:dataPoints][1]][:E]       # total energy consumption (in Ws)

            merge!(BSsModified, Dict(:acceleration=>accelerationSection))
            csModified[:E] = csModified[:E] + accelerationSection[:E]
            csModified[:t] = csModified[:t] + accelerationSection[:t]

        else
            # The acceleration section is only one step. This step is removed and if there is a clearing section it will be combined with the new cruising section.
            energySavingStartId=get(BSsOriginal, :clearing, get(BSsOriginal, :acceleration, Dict(:dataPoints =>[0])))[:dataPoints][1]
        end

        # TODO: should v_peak be reduced or is it enough to pop the data points?
            #    characteristicSection[:v_peak]=drivingCourse[end][:v]      # setting v_peak to the last data point's velocity which is the highest reachable value in this characteristic section

        # copy the drivingCourse till the beginning of energy saving
        drivingCourseModified = copy(drivingCourse[1:energySavingStartId])          # List of data points till the start of energy saving

        s_braking = calcBrakingDistance(drivingCourseModified[end][:v], csModified[:v_exit], train[:a_braking])
        s_cruising = csModified[:s_exit]-drivingCourseModified[end][:s]-s_braking

        if s_cruising > 1/10^approximationLevel
        # 01/09 old if s_cruising > 0.001
            (csModified, drivingCourseModified)=addCruisingSection!(csModified, drivingCourseModified, s_cruising, settings, train, allCSs, "cruising")
        end #if


        # s_brakingAfterCruising=ceil((csModified[:v_exit]^2-drivingCourseModified[end][:v]^2)/2/train[:a_braking], digits=10) #  TODO: check if s_braking and s_brakingAfterCruising are really always the same
        if drivingCourseModified[end][:v]>csModified[:v_exit]
            #(csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings[:massModel], train, allCSs)
            (csModified, drivingCourseModified)=addBrakingSection!(csModified, drivingCourseModified, settings, train, allCSs)

        elseif drivingCourseModified[end][:s]<csModified[:s_exit]
            if (csModified[:s_exit]-drivingCourseModified[end][:s])>0.001
                # if (csModified[:s_exit]-drivingCourseModified[end][:s])>10^(-approximationLevel)
            #    println("INFO: The end of new CS",csModified[:id]," is not reached while saving energy with lowering v_peak.")
            #    println("      Therefore the calculation of this method can not continue for this CS.")
            # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
                return (csOriginal, drivingCourse, false)
            end
            println("WARNING: The end of new CS",csModified[:id]," is not reached while saving energy with lowering v_peak.")
            println("         Therefore   s=",drivingCourseModified[end][:s]," will be set s_exit=",csModified[:s_exit]," because the difference is only ",csModified[:s_exit]-drivingCourseModified[end][:s]," m.")
            println("          v=",drivingCourseModified[end][:v]," m/s   v_exit=",csOriginal[:v_exit] ," m/s")

            drivingCourseModified[end][:s] = csModified[:s_exit]       # rounding up to s_exit
            drivingCourseModified[end][:Δs] = drivingCourseModified[end][:s] - drivingCourseModified[end-1][:s]
        end #if

        if t_recoveryAvailable >= csModified[:t] - csOriginal[:t]

            return (csModified, drivingCourseModified, true)
        else # time loss is to high. so there is no energy saving modification for this CS with the available recovery time
            # 09/06 old: else # time loss is to high and the CS has to be calculated again with larger acceleration section (so with a smaller reduction of the acceleration section)
            # 09/06 old: accelerationReduction=min(accelerationReduction/10, csModified[:v_peak]-csModified[:v_entry], csModified[:v_peak]-csModified[:v_exit])
                # TODO: just return false or take smaller steps?

            # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
            return (csOriginal, drivingCourse, false)
        end

    # 09/06 old:   end #while
    #
    # 09/06 old:   # there is no energy saving modification for this CS with the available recovery time
    #  old:   return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
    # 12/29 new, now not with empty but with original CS and DC: return (csOriginal, drivingCourse, false)


    else
        # there is no energy saving modification for this CS because v_peak can not be lowered below v_entry or v_exit or because there is no acceleration section that can be transformed into a cruising section
        # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
        return (csOriginal, drivingCourse, false)
    end #if haskey
end # function decreaseMaximumVelocity

# combination of method 1 and method 2
function combineEnergySavingMethods(csOriginal::Dict, drivingCourse::Vector{Dict}, settings::Dict, train::Dict, allCSs::Vector{Dict}, t_recoveryAvailable::AbstractFloat)
    BSsOriginal = csOriginal[:behaviorSections]
 #    if haskey(BSsOriginal, :acceleration) && (haskey(BSsOriginal, :braking) || haskey(BSsOriginal, :coasting)) && csOriginal[:v_peak]>csOriginal[:v_entry] && csOriginal[:v_peak]>csOriginal[:v_exit]
    if haskey(BSsOriginal, :acceleration) && (haskey(BSsOriginal, :braking) || haskey(BSsOriginal, :coasting)) && drivingCourse[get(BSsOriginal, :acceleration, Dict(:dataPoints =>[0]))[:dataPoints][end]][:v] > max(csOriginal[:v_entry], csOriginal[:v_exit])
        # copy the characteristic section
        csCombined = copyCharacteristicSection(csOriginal)

        # copy the drivingCourse
        drivingCourseCombined = copy(drivingCourse)

        ΔE=0.0      # saved energy (in Ws)
        Δt=0.0      # time loss (in s)
        #while (Δt<t_recoveryAvailable && ΔE<=0.0)
        while (Δt<t_recoveryAvailable && (Δt<=0.0 || ΔE<=0.0)) #ΔE<=0.0)
            (csCombined, drivingCourseCombined, newMaximumVelocity)=decreaseMaximumVelocity(csCombined, drivingCourseCombined, settings, train, allCSs, t_recoveryAvailable)
            if newMaximumVelocity
                ΔE=csOriginal[:E]-csCombined[:E]      # saved energy (in Ws)
                Δt=csCombined[:t]-csOriginal[:t]      # time loss (in s)

                while (haskey(BSsOriginal, :cruising) && (Δt<=0.0 || ΔE<=0.0)) #Δt<=0.0) #&& ΔE<=0.0) # && Δt<=0.0)
                    (csCombined, drivingCourseCombined, newCoasting)=increaseCoastingSection(csCombined, drivingCourseCombined, settings, train, allCSs, t_recoveryAvailable-Δt)
                    if newCoasting
                        ΔE=csOriginal[:E]-csCombined[:E]      # saved energy (in Ws)
                        Δt=csCombined[:t]-csOriginal[:t]      # time loss (in s)
                    else
                        break
                    end # if newCoasting
                end # while haskey
            else
                # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
                return (csOriginal, drivingCourse, false)
            end # if newMaximumVelocity
        end # while
        return (csCombined, drivingCourseCombined, (ΔE>0.0))# && Δt>0.0))
    else
        # there is no energy saving modification for this CS because v_peak can not be lowered below v_entry or v_exit or because there is no acceleration section and braking section or coasting section that can be transformed into a cruising section or coasting section
        # 12/29 old, now not with empty but with original CS and DC: return (Dict(), [], false)    # TODO: Does the empty CS-Dict need default attributes?
        return (csOriginal, drivingCourse, false)
    end #if
end #function combineEnergySavingMethods

end #module EnergySaving
