#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

# INFO: AdditionalOutput should not be used because it is not completed yet. It was used to show first results during development.
# TODO: It has to be optimized so that the created plots and printed information is clear and understandable.

module AdditionalOutput

using Plots

export plotResults, plotDrivingCourse, printImportantValues, printSectionInformation

function plotResults(output::Dict)
    opModeMinTime = output[:settings][:operationModeMinimumRunningTime]
    opModeMinEnergy = output[:settings][:operationModeMinimumEnergyConsumption]

    if opModeMinTime == true && opModeMinEnergy == true
        plotDrivingCourse(output[:drivingCourseMinimumRunningTime], output[:drivingCourseMinimumEnergyConsumption])
    elseif opModeMinTime == true
        plotDrivingCourse(output[:drivingCourseMinimumRunningTime])
    elseif  opModeMinEnergy == true
        plotDrivingCourse(output[:drivingCourseMinimumEnergyConsumption])
    else
        output[:settings][:detailOfOutput] == "everything" && println("No Output was demanded. So no plot is created.")
    end

    return true
end #function plotResults

function plotResults(drivingCourse::Vector{Dict})
    plotDrivingCourse(drivingCourse)

    return true
end #function plotResults

function plotResults(singleValue::AbstractFloat)
    println("Not able to plot the single value ",singleValue)

    return false
end #function plotResults

function plotDrivingCourse(drivingCourse::Vector{Dict})
    a=[]
    E=[]
    s=[]
    t=[]
    v=[]
    for i in 1:length(drivingCourse)
        push!(a, drivingCourse[i][:a])
        push!(E, drivingCourse[i][:E])
        push!(s, drivingCourse[i][:s])
        push!(t, drivingCourse[i][:t])
        push!(v, drivingCourse[i][:v])
    end #for

#    p1=plot([s], [v], title = "v in m/s", label = ["v"], xlabel = "s in m")
    p1=plot([s/1000], [v*3.6], title = "v in km/h", label = ["v"], xlabel = "s in km")

#    p2=plot([t], [v], title = "v in m/s", label = ["v"], xlabel = "t in s")
    p2=plot([t/60], [v*3.6], title = "v in km/h", label = ["v"], xlabel = "t in min")

 #   p3=plot([s], [t], title = "t in s", label = ["t"], xlabel = "s in m")

 #   p4=plot([t], [s], title = "s in m", label = ["s"], xlabel = "t in s")

    #p5=plot([s], [E], title = "E in Ws", label = ["E"], xlabel = "s in m")
    p5=plot([s/1000], [E], title = "E in Ws", label = ["E"], xlabel = "s in km")

    #p6=plot([t], [E], title = "E in Ws", label = ["E"], xlabel = "t in s")
    p6=plot([t/60], [E], title = "E in Ws", label = ["E"], xlabel = "t in min")

    all=plot(p1, p2, p5, p6, layout = (2, 2), legend = false)
#=
 #   p5=plot([s], [E], title = "E in Ws", label = ["E"], xlabel = "s in m")

 #   p6=plot([t], [E], title = "E in Ws", label = ["E"], xlabel = "t in s")

    all=plot(p1, p2,  layout = (1, 2), legend = false)=#
 #   all=plot(p1, p2, p3, p4, p5, p6, layout = (3, 2), legend = false)
    display(all)
    println("Plots for different variables have been created.")
end #function plotDrivingCourse

function plotDrivingCourse(drivingCourseMinimumRunningTime::Vector{Dict},drivingCourseMinimumEnergyConsumption::Vector{Dict})
    a_minTime=[]
    E_minTime=[]
    s_minTime=[]
    t_minTime=[]
    v_minTime=[]
    for i in 1:length(drivingCourseMinimumRunningTime)
        push!(a_minTime, drivingCourseMinimumRunningTime[i][:a])
        push!(E_minTime, drivingCourseMinimumRunningTime[i][:E])
        push!(s_minTime, drivingCourseMinimumRunningTime[i][:s])
        push!(t_minTime, drivingCourseMinimumRunningTime[i][:t])
        push!(v_minTime, drivingCourseMinimumRunningTime[i][:v])
    end #for

    a_minEnergy=[]
    E_minEnergy=[]
    s_minEnergy=[]
    t_minEnergy=[]
    v_minEnergy=[]
    for i in 1:length(drivingCourseMinimumEnergyConsumption)
        push!(a_minEnergy, drivingCourseMinimumEnergyConsumption[i][:a])
        push!(E_minEnergy, drivingCourseMinimumEnergyConsumption[i][:E])
        push!(s_minEnergy, drivingCourseMinimumEnergyConsumption[i][:s])
        push!(t_minEnergy, drivingCourseMinimumEnergyConsumption[i][:t])
        push!(v_minEnergy, drivingCourseMinimumEnergyConsumption[i][:v])
    end #for

    p1=plot([s_minTime,s_minEnergy],
        [v_minTime,v_minEnergy],
        title = "v in m/s",
        label = ["v for t_min" "v for E_min"],
        xlabel = "s in m")# lw = 3)

    p2=plot([t_minTime,t_minEnergy],
        [v_minTime,v_minEnergy],
        title = "v in m/s",
        label = ["v for t_min" "v for E_min"],
        xlabel = "t in s")

 #   p3=plot([s_minTime,s_minEnergy],
 #       [t_minTime,t_minEnergy],
 #       title = "t in s",
 #       label = ["t for t_min" "t for E_min"],
 #       xlabel = "s in m")

 #   p4=plot([t_minTime,t_minEnergy],
 #       [s_minTime,s_minEnergy],
 #       title = "s in m",
 #       label = ["s for t_min" "s for E_min"],
 #       xlabel = "t in s")

    p5=plot([s_minTime,s_minEnergy],
        [E_minTime,E_minEnergy],
        title = "E in Ws",
        label = ["E for t_min" "E for E_min"],
        xlabel = "s in m")

    p6=plot([t_minTime,t_minEnergy],
        [E_minTime,E_minEnergy],
        title = "E in Ws",
        label = ["E for t_min" "E for E_min"],
        xlabel = "t in s")

 #   all=plot(p1, p2, p3, p4, p5, p6, layout = (3, 2), legend = false)
    all=plot(p1, p2, p5, p6, layout = (2, 2), legend = false)
    display(all)
    println("Plots for different variables have been created.")
end #function plotDrivingCourse

function printImportantValues(dataPoints::Vector{Dict})
    println("i      behavior                 s in m                 v in km/h                t in min               a in m/s^2                F_R in k N                F_T in k N                E in k Wh")
    for i in 1:length(dataPoints)
        println(dataPoints[i][:i],".   ",dataPoints[i][:behavior],"  ",dataPoints[i][:s],"  ",dataPoints[i][:v]*3.6,"  ",dataPoints[i][:t]/60,"  ",dataPoints[i][:a],"  ",dataPoints[i][:F_R]/1000,"  ",dataPoints[i][:F_T]/1000,"  ",dataPoints[i][:E]/3600/1000)
    end #for
    println("i      behavior                 s in m                 v in km/h                t in min               a in m/s^2                F_R in k N                F_T in k N                E in k Wh")
end #function printImportantValues

function printSectionInformation(movingSection::Dict)
    CSs::Vector{Dict} = movingSection[:characteristicSections]

    println("MS   with length=", movingSection[:length]," with t=", movingSection[:t])
    #allBSs=[:breakFree, :clearing, :accelerating,  :clearing2, :accelerating2,  :clearing3, :accelerating3, :cruising, :downhillBraking, :diminishing, :coasting, :braking, :standstill]
    for csId in 1:length(CSs)
        println("CS ",csId,"  with length=", CSs[csId][:length]," with t=", CSs[csId][:t])
    #    for bs in 1: length(allBSs)
    #        if haskey(CSs[csId][:behaviorSections], allBSs[bs])
    #            println("BS ",allBSs[bs], "   with s_entry=", CSs[csId][:behaviorSections][allBSs[bs]][:s_entry], "   and length=",CSs[csId][:behaviorSections][allBSs[bs]][:length])  # and t=", CSs[csId][:behaviorSections][allBSs[bs]][:t])
    #    #        for point in 1:length(CSs[csId][:behaviorSections][allBSs[bs]][:dataPoints])
    #    #            println(CSs[csId][:behaviorSections][allBSs[bs]][:dataPoints][point])
    #    #        end
    #        end #if
    #    end #for

        tempBSs = collect(keys(CSs[csId][:behaviorSections]))
        while length(tempBSs) > 0
            currentBS = :default
            for bs in 1: length(tempBSs)
                if currentBS == :default
                    currentBS = tempBSs[bs]
                else
                    if CSs[csId][:behaviorSections][currentBS][:s_entry] > CSs[csId][:behaviorSections][tempBSs[bs]][:s_entry]
                        currentBS = tempBSs[bs]
                    end
                end
            end #for

            println("BS ",currentBS, "   with s_entry=", CSs[csId][:behaviorSections][currentBS][:s_entry], "   and length=",CSs[csId][:behaviorSections][currentBS][:length])  # and t=", CSs[csId][:behaviorSections][currentBS][:t])
    #        for point in 1:length(CSs[csId][:behaviorSections][currentBS][:dataPoints])
    #            println(CSs[csId][:behaviorSections][currentBS][:dataPoints][point])
    #        end

            newTempBSs = []
            for bs in 1: length(tempBSs)
                if currentBS != tempBSs[bs]
                    push!(newTempBSs, tempBSs[bs])
                end #if
            end #for
            tempBSs = newTempBSs
        end #while
    end #for
end #function printSectionInformation

end # module AdditionalOutput
