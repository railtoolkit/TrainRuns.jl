module Output

using ..types
using Plots
using CSV, DataFrames, Dates

export createOutput
export plotDrivingCourse, printImportantValues, printSectionInformation # functions for showing results during the development

function createOutput(settings::Dict, pathName::String, trainName::String, drivingCourse::Vector{DataPoint}, movingSection::Dict)
    # method of function createOutput for one operation mode
    if settings[:typeOfOutput] == "CSV"
        return createOutputCsv(settings, pathName, trainName, drivingCourse, movingSection)
    else
        return createOutputDict(settings, pathName, trainName, drivingCourse, movingSection)
    end
end # funtion createOutput

function createOutput(settings::Dict, pathName::String, trainName::String, drivingCourseMinimumRunningTime::Vector{DataPoint}, movingSectionMinimumRunningTime::Dict, drivingCourseMinimumEnergyConsumption::Vector{DataPoint}, movingSectionMinimumEnergyConsumption::Dict)
    # method of function createOutput for two operation modes
    if settings[:typeOfOutput] == "CSV"
        return createOutputCsv(settings, pathName, trainName, drivingCourseMinimumRunningTime, movingSectionMinimumRunningTime, drivingCourseMinimumEnergyConsumption, movingSectionMinimumEnergyConsumption)
    else
        return createOutputDict(settings, pathName, trainName, drivingCourseMinimumRunningTime, movingSectionMinimumRunningTime, drivingCourseMinimumEnergyConsumption, movingSectionMinimumEnergyConsumption)
    end
end # funtion createOutput


function createOutputDict(settings::Dict, pathName::String, trainName::String, drivingCourse::Vector{DataPoint}, movingSection::Dict)
    # method of function createOutputDict for one operation mode
    if settings[:operationModeMinimumRunningTime]
        if settings[:operationModeMinimumEnergyConsumption]
            operationMode="minimum running time and minimum energy consumption"
        else
            operationMode="minimum running time"
        end
    else
        if settings[:operationModeMinimumEnergyConsumption]
            operationMode="minimum energy consumption"
        else
            # should never be the case
            println("No Output was demanded. So no output dictionary is created")
            return
        end
    end

    outputDict=Dict([(:pathName, pathName), (:trainName, trainName), (:operationMode, operationMode), (:massModel, settings[:massModel]), (:stepVariable, settings[:stepVariable]), (:stepSize, settings[:stepSize])])
    #outputDict=Dict([("path name", pathName), ("train name", trainName), ("operation mode", operationMode), ("mass model", settings[:massModel]), ("step variable", settings[:stepVariable]), ("step size", settings[:stepSize])])

    # creating an output array
    outputArray=Array{Any, 1}[]
    if settings[:detailOfOutput]=="minimal"

        push!(outputArray, ["s (in m)", "t (in s)","E (in Ws)"]) #  push header to outputArray

        row=[movingSection[:length], movingSection[:t], movingSection[:E]]
        push!(outputArray, row)                                                     # push row to outputArray
    elseif settings[:detailOfOutput]=="driving course"
        push!(outputArray, ["i", "behavior", "Δs (in m)", "s (in m)", "Δt (in s)","t (in s)","Δv (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_consist (in N)", "ΔW (in Ws)","W (in Ws)","ΔE (in  Ws)","E (in Ws)","a (in m/s^2)"]) # push header to outputArray
        for point in drivingCourse
            row=[point.i, point.behavior, point.Δs, point.s, point.Δt, point.t, point.Δv, point.v, point.F_T, point.F_R, point.R_path, point.R_train, point.R_traction, point.R_consist, point.ΔW, point.W, point.ΔE, point.E, point.a]
            push!(outputArray, row)             # push row to outputArray
        end
    end

    if length(movingSection[:energySavingModifications])>0   # if the moving section hast energy saving modifications the moving section is modified for minimum energy consumption
        merge!(outputDict, Dict(:outputArrayMinimumEnergyConsumption => outputArray))
    else
        merge!(outputDict, Dict(:outputArrayMinimumRunningTime => outputArray))
    end

    println("The output dictionary has been created for ", operationMode,".")
    return outputDict
end # function createOutputDict

function createOutputDict(settings::Dict, pathName::String, trainName::String, drivingCourseMinimumRunningTime::Vector{DataPoint}, movingSectionMinimumRunningTime::Dict, drivingCourseMinimumEnergyConsumption::Vector{DataPoint}, movingSectionMinimumEnergyConsumption::Dict)
    # method of function createOutputDict for two operation modes
    if settings[:operationModeMinimumRunningTime]
        outputDict=createOutputDict(settings, pathName, trainName, drivingCourseMinimumRunningTime, movingSectionMinimumRunningTime)

        if settings[:operationModeMinimumEnergyConsumption]
            # creating the second output array
            outputArrayMinimumEnergyConsumption=Array{Any, 1}[]
            if settings[:detailOfOutput]=="minimal"
                push!(outputArrayMinimumEnergyConsumption, ["s (in m)", "t (in s)","E (in Ws)"])  # push header to outputArrayMinimumEnergyConsumption
                row=[movingSectionMinimumEnergyConsumption[:length], movingSectionMinimumEnergyConsumption[:t], movingSectionMinimumEnergyConsumption[:E]]
                push!(outputArrayMinimumEnergyConsumption, row)                                                     # push row to outputArrayMinimumEnergyConsumption
            elseif settings[:detailOfOutput]=="driving course"
                push!(outputArrayMinimumEnergyConsumption, ["i", "behavior", "Δs (in m)", "s (in m)", "Δt (in s)","t (in s)","Δv (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_consist (in N)", "ΔW (in Ws)","W (in Ws)","ΔE (in  Ws)","E (in Ws)","a (in m/s^2)"]) # push header to outputArrayMinimumEnergyConsumption
                for point in drivingCourseMinimumEnergyConsumption
                    row=[point.i, point.behavior, point.Δs, point.s, point.Δt, point.t, point.Δv, point.v, point.F_T, point.F_R, point.R_path, point.R_train, point.R_traction, point.R_consist, point.ΔW, point.W, point.ΔE, point.E, point.a]
                    push!(outputArrayMinimumEnergyConsumption, row)             # push row to outputArrayMinimumEnergyConsumption
                end
            end

            merge!(outputDict, Dict(:outputArrayMinimumEnergyConsumption => outputArrayMinimumEnergyConsumption))
        end
    else
        outputDict=createOutputDict(settings, pathName, trainName, drivingCourseMinimumEnergyConsumption, movingSectionMinimumEnergyConsumption)
    end

        # println("The output dictionary has been created for minimum running time and minimum energy consumption.")
    return outputDict
end # function createOutputDict


function createOutputCsv(settings::Dict, pathName::String, trainName::String, drivingCourse::Vector{DataPoint}, movingSection::Dict)
    # method of function createOutputDict for one operation mode
    outputDict=createOutputDict(settings, pathName, trainName, drivingCourse, movingSection)

    if length(movingSection[:energySavingModifications])>0   # if the moving section hast energy saving modifications the moving section is modified for minimum energy consumption
        operationMode="minimum energy consumption"
        outputArray=:outputArrayMinimumEnergyConsumption
        date = Dates.now()
        dateString=Dates.format(date, "yyyy-mm-dd_HH.MM.SS")
        csvFilePath=settings[:csvDirectory]*"/"*dateString*"_MinimumEnergyConsumption.csv"
    else
        operationMode="minimum running time"
        outputArray=:outputArrayMinimumRunningTime
        date = Dates.now()
        dateString=Dates.format(date, "yyyy-mm-dd_HH.MM.SS")
        csvFilePath=settings[:csvDirectory]*"/"*dateString*"_MinimumRunningTime.csv"
    end

    # creating information block
    infoColumns=Array{Any,1}[]
    push!(infoColumns, ["path name", "train name", "operation mode", "mass model", "step variable", "step size", ""])
    push!(infoColumns, [pathName, trainName, operationMode, settings[:massModel], settings[:stepVariable], string(settings[:stepSize]), ""])
    for column in 3:length(outputDict[outputArray][1])
        push!(infoColumns, ["", "", "", "", "", "", ""])
    end # for

    allColumns=Array{Any,1}[]
    if settings[:detailOfOutput]=="minimal"
        header=outputDict[:outputArrayMinimumRunningTime][1]
    elseif settings[:detailOfOutput]=="driving course"
        header=["i", "behavior", "Delta s (in m)", "s (in m)", "Delta t (in s)","t (in s)","Delta v (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_consist (in N)"," Delta W (in Ws)","W (in Ws)","Delta E (in  Ws)","E (in Ws)","a (in m/s^2)"]
    end
    for column in 1:length(outputDict[outputArray][1])
        push!(infoColumns[column], header[column])
        for row in outputDict[outputArray][2:end]
            push!(infoColumns[column], row[column])
        end
        push!(allColumns, infoColumns[column])
    end # for


    # combining the columns in a data frame and saving it as a CSV-file at csvDirectory
    if settings[:detailOfOutput]=="minimal"
        df=DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3])
    elseif settings[:detailOfOutput]=="driving course"
        df=DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3], c4=allColumns[4], c5=allColumns[5], c6=allColumns[6], c7=allColumns[7], c8=allColumns[8], c9=allColumns[9], c10=allColumns[10], c11=allColumns[11], c12=allColumns[12], c13=allColumns[13], c14=allColumns[14], c15=allColumns[15], c16=allColumns[16], c17=allColumns[17], c18=allColumns[18])
    end
    CSV.write(csvFilePath, df, header=false)

    println("The output CSV file has been created for ",operationMode," at ",csvFilePath)
    return outputDict
end #function createOutputCsv

function createOutputCsv(settings::Dict, pathName::String, trainName::String, drivingCourseMinimumRunningTime::Vector{DataPoint}, movingSectionMinimumRunningTime::Dict, drivingCourseMinimumEnergyConsumption::Vector{DataPoint}, movingSectionMinimumEnergyConsumption::Dict)
    # method of function createOutputDict for two operation modes
    outputDict=createOutputDict(settings, pathName, trainName, drivingCourseMinimumRunningTime, movingSectionMinimumRunningTime, drivingCourseMinimumEnergyConsumption, movingSectionMinimumEnergyConsumption)

    if settings[:operationModeMinimumRunningTime]
        #creating information block
        infoColumns=Array{Any,1}[]
        push!(infoColumns, ["path name", "train name", "operation mode", "mass model", "step variable", "step size", ""])
        #push!(infoColumns, [pathName, trainName, "minimum running time", settings[:massModel], settings[:stepVariable], settings[:stepSize], ""])
        push!(infoColumns, [pathName, trainName, "minimum running time", settings[:massModel], settings[:stepVariable], string(settings[:stepSize]), ""])

        for column in 3:length(outputDict[:outputArrayMinimumRunningTime][1])
            push!(infoColumns, ["", "", "", "", "", "", ""])
        #    println("push wird ausgeführt")
        end # for

        allColumns=Array{Any,1}[]
        if settings[:detailOfOutput]=="minimal"
            header=outputDict[:outputArrayMinimumRunningTime][1]
        elseif settings[:detailOfOutput]=="driving course"
            header=["i", "behavior", "Delta s (in m)", "s (in m)", "Delta t (in s)","t (in s)","Delta v (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_consist (in N)"," Delta W (in Ws)","W (in Ws)","Delta E (in  Ws)","E (in Ws)","a (in m/s^2)"]
        end

        for column in 1:length(outputDict[:outputArrayMinimumRunningTime][1])
            push!(infoColumns[column], header[column])
            for row in outputDict[:outputArrayMinimumRunningTime][2:end]
                push!(infoColumns[column], row[column])
            end
            push!(allColumns, infoColumns[column])
        end # for

        #combining the columns in a data frame and saving it as a CSV-file at csvDirectory
        if settings[:detailOfOutput]=="minimal"
            df=DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3])
        elseif settings[:detailOfOutput]=="driving course"
            df=DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3], c4=allColumns[4], c5=allColumns[5], c6=allColumns[6], c7=allColumns[7], c8=allColumns[8], c9=allColumns[9], c10=allColumns[10], c11=allColumns[11], c12=allColumns[12], c13=allColumns[13], c14=allColumns[14], c15=allColumns[15], c16=allColumns[16], c17=allColumns[17], c18=allColumns[18])
        end
        date = Dates.now()
        dateString=Dates.format(date, "yyyy-mm-dd_HH.MM.SS")
        csvFilePath=settings[:csvDirectory]*"/"*dateString*"_dataMinimumRunningTime.csv"
        CSV.write(csvFilePath, df, header=false)
        println("The output CSV file has been created for minimum running time at ",csvFilePath)
    end #if settings[:operationModeMinimumRunningTime]


    if settings[:operationModeMinimumEnergyConsumption]
        #creating information block
        infoColumns=Array{Any,1}[]
        push!(infoColumns, ["path name", "train name", "operation mode", "mass model", "step variable", "step size", ""])
        push!(infoColumns, [pathName, trainName, "minimum energy consumption", settings[:massModel], settings[:stepVariable], settings[:stepSize], ""])
        for column in 3:length(outputDict[:outputArrayMinimumEnergyConsumption][1])
            push!(infoColumns, ["", "", "", "", "", "", ""])
        end # for

        allColumns=Array{Any,1}[]
        if settings[:detailOfOutput]=="minimal"
            header=outputDict[:outputArrayMinimumRunningTime][1]
        elseif settings[:detailOfOutput]=="driving course"
            header=["i", "behavior", "Delta s (in m)", "s (in m)", "Delta t (in s)","t (in s)","Delta v (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_consist (in N)"," Delta W (in Ws)","W (in Ws)","Delta E (in  Ws)","E (in Ws)","a (in m/s^2)"]
        end

        for column in 1:length(outputDict[:outputArrayMinimumEnergyConsumption][1])
            push!(infoColumns[column], header[column])
            for row in outputDict[:outputArrayMinimumEnergyConsumption][2:end]
                push!(infoColumns[column], row[column])
            end
            push!(allColumns, infoColumns[column])
        end # for

        #combining the columns in a data frame
        if settings[:detailOfOutput]=="minimal"
            df=DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3])
        elseif settings[:detailOfOutput]=="driving course"
            df=DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3], c4=allColumns[4], c5=allColumns[5], c6=allColumns[6], c7=allColumns[7], c8=allColumns[8], c9=allColumns[9], c10=allColumns[10], c11=allColumns[11], c12=allColumns[12], c13=allColumns[13], c14=allColumns[14], c15=allColumns[15], c16=allColumns[16], c17=allColumns[17], c18=allColumns[18])
        end

        # creating a CSV-file at csvDirectory
        date = Dates.now()
        dateString=Dates.format(date, "yyyy-mm-dd_HH.MM.SS")
        csvFilePath=settings[:csvDirectory]*"/"*dateString*"_dataMinimumEnergyConsumption.csv"
        CSV.write(csvFilePath, df, header=false)
        println("The output CSV file has been created for minimum energy consumption at ",csvFilePath)
    end # if settings[:operationModeMinimumEnergyConsumption]

    return outputDict
end #function createOutputCsv



function printImportantValues(drivingCourse::Vector{DataPoint})
    println("i      behavior                 s in m                 v in km/h                t in min               a in m/s^2                F_R in k N                F_T in k N                E in k Wh")
    for i in 1:length(drivingCourse)
        println(drivingCourse[i].i,".   ",drivingCourse[i].s,"  ",drivingCourse[i].v*3.6,"  ",drivingCourse[i].t/60,"  ",drivingCourse[i].a,"  ",drivingCourse[i].F_R/1000,"  ",drivingCourse[i].F_T/1000,"  ",drivingCourse[i].E/3600/1000)
    end #for
    println("i      behavior                 s in m                 v in km/h                t in min               a in m/s^2                F_R in k N                F_T in k N                E in k Wh")
end #function printImportantValues

function printSectionInformation(movingSection::Dict)
    CSs::Vector{CharacteristicSection} = movingSection[:characteristicSections]

    println("MS   mit length=", movingSection[:length]," mit t=", movingSection[:t])
    allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :cruisingAfterCoasting, :braking, :standStill]
    for csId in 1:length(CSs)
        println("CS ",csId,"  mit length=", CSs[csId].length," mit t=", CSs[csId].t)
        for bs in 1: length(allBs)
            if haskey(CSs[csId].behaviorSections, allBs[bs])
                println("BS ",allBs[bs], "   mit s_entry=", CSs[csId].behaviorSections[allBs[bs]].s_entry, "   und t=", CSs[csId].behaviorSections[allBs[bs]].t)
        #        for point in 1:length(CSs[csId].behaviorSections[allBs[bs]].dataPoints)
        #            println(CSs[csId].behaviorSections[allBs[bs]].dataPoints[point])
        #        end
            end #if
        end #for
    end #for
end #function printSectionInformation

function plotDrivingCourse(drivingCourse::Vector{DataPoint})
    a=[]
    E=[]
    s=[]
    t=[]
    v=[]
    for i in 1:length(drivingCourse)
        push!(a, drivingCourse[i].a)
        push!(E, drivingCourse[i].E)
        push!(s, drivingCourse[i].s)
        push!(t, drivingCourse[i].t)
        push!(v, drivingCourse[i].v)
    end #for

    p1=plot([s], [v], title = "v in m/s", label = ["v"], xlabel = "s in m")

    p2=plot([t], [v], title = "v in m/s", label = ["v"], xlabel = "t in s")

 #   p3=plot([s], [t], title = "t in s", label = ["t"], xlabel = "s in m")

 #   p4=plot([t], [s], title = "s in m", label = ["s"], xlabel = "t in s")

    p5=plot([s], [E], title = "E in Ws", label = ["E"], xlabel = "s in m")

    p6=plot([t], [E], title = "E in Ws", label = ["E"], xlabel = "t in s")

    all=plot(p1, p2, p5, p6, layout = (2, 2), legend = false)
 #   all=plot(p1, p2, p3, p4, p5, p6, layout = (3, 2), legend = false)
    display(all)
    println("Plots for different variables have been created.")
end #function plotDrivingCourse

function plotDrivingCourse(drivingCourseMinimumRunningTime::Vector{DataPoint},drivingCourseMinimumEnergyConsumption::Vector{DataPoint})
    a_minTime=[]
    E_minTime=[]
    s_minTime=[]
    t_minTime=[]
    v_minTime=[]
    for i in 1:length(drivingCourseMinimumRunningTime)
        push!(a_minTime, drivingCourseMinimumRunningTime[i].a)
        push!(E_minTime, drivingCourseMinimumRunningTime[i].E)
        push!(s_minTime, drivingCourseMinimumRunningTime[i].s)
        push!(t_minTime, drivingCourseMinimumRunningTime[i].t)
        push!(v_minTime, drivingCourseMinimumRunningTime[i].v)
    end #for

    a_minEnergy=[]
    E_minEnergy=[]
    s_minEnergy=[]
    t_minEnergy=[]
    v_minEnergy=[]
    for i in 1:length(drivingCourseMinimumEnergyConsumption)
        push!(a_minEnergy, drivingCourseMinimumEnergyConsumption[i].a)
        push!(E_minEnergy, drivingCourseMinimumEnergyConsumption[i].E)
        push!(s_minEnergy, drivingCourseMinimumEnergyConsumption[i].s)
        push!(t_minEnergy, drivingCourseMinimumEnergyConsumption[i].t)
        push!(v_minEnergy, drivingCourseMinimumEnergyConsumption[i].v)
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

end # module Output
