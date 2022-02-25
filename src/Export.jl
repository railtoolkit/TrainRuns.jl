#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

module Export

using CSV, DataFrames, Dates

export exportToCsv

function exportToCsv(output::Dict)
    if output[:settings][:typeOfOutput] == "CSV"
        pathName = output[:path][:name]
        trainName = output[:train][:name]

        if output[:settings][:operationModeMinimumRunningTime] == true
            operationMode = "minimum running time"
            if output[:settings][:detailOfOutput] == "points of interest"
                dataPointsToExport = output[:pointsOfInterestMinimumRunningTime]
            else
                dataPointsToExport = output[:drivingCourseMinimumRunningTime]
            end
            createCsvFile(output[:movingSectionMinimumRunningTime], dataPointsToExport, operationMode, pathName, trainName, output[:settings])
        end
        if output[:settings][:operationModeMinimumEnergyConsumption] == true
            operationMode = "minimum energy consumption"
            if output[:settings][:detailOfOutput] == "points of interest"
                dataPointsToExport = output[:pointsOfInterestMinimumEnergyConsumption]
            else
                dataPointsToExport = output[:drivingCourseMinimumEnergyConsumption]
            end
            createCsvFile(output[:movingSectionMinimumEnergyConsumption], dataPointsToExport, operationMode, pathName, trainName, output[:settings])
        end
        return true
    end
    return false
end #function exportToCsv

function createCsvFile(movingSection::Dict, dataPointsToExport::Vector{Dict}, operationMode::String, pathName::String, trainName::String, settings::Dict)
    detailOfOutput = settings[:detailOfOutput]

    massModel = settings[:massModel]
    stepVariable = settings[:stepVariable]
    stepSize = string(settings[:stepSize])

    # create accumulated data block
    accumulatedData = Array{Any, 1}[]
    if detailOfOutput == "minimal"
        push!(accumulatedData, ["s (in m)", "t (in s)","E (in Ws)"])                     # push header to accumulatedData
        row = [movingSection[:length], movingSection[:t], movingSection[:E]]
        push!(accumulatedData, row)                                                      # push row to accumulatedData
    elseif detailOfOutput == "driving course" || detailOfOutput == "points of interest"
        push!(accumulatedData, ["i", "behavior", "Δs (in m)", "s (in m)", "Δt (in s)","t (in s)","Δv (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_wagons (in N)", "ΔW (in Ws)","W (in Ws)","ΔE (in  Ws)","E (in Ws)","a (in m/s^2)"]) # push header to accumulatedData
        for point in dataPointsToExport
            row = [point[:i], point[:behavior], point[:Δs], point[:s], point[:Δt], point[:t], point[:Δv], point[:v], point[:F_T], point[:F_R], point[:R_path], point[:R_train], point[:R_traction], point[:R_wagons], point[:ΔW], point[:W], point[:ΔE], point[:E], point[:a]]
            push!(accumulatedData, row)             # push row to accumulatedData
        end
    end

    #create information block
    allColumns=Array{Any,1}[]
    push!(allColumns, ["path name", "train name", "operation mode", "mass model", "step variable", "step size", ""])
    push!(allColumns, [pathName, trainName, operationMode, massModel, stepVariable, stepSize, ""])
    for column in 3:length(accumulatedData[1])
        push!(allColumns, ["", "", "", "", "", "", ""])
    end # for

    # add driving data to the array
    header = accumulatedData[1]
    for column in 1:length(accumulatedData[1])
        push!(allColumns[column], header[column])
        for row in accumulatedData[2:end]
            push!(allColumns[column], row[column])
        end
    end # for

    # combine the columns in a data frame and saving it as a CSV-file at csvDirectory
    if detailOfOutput == "minimal"
        df = DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3])
    elseif detailOfOutput=="driving course" || detailOfOutput == "points of interest"
        df = DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3], c4=allColumns[4], c5=allColumns[5], c6=allColumns[6], c7=allColumns[7], c8=allColumns[8], c9=allColumns[9], c10=allColumns[10], c11=allColumns[11], c12=allColumns[12], c13=allColumns[13], c14=allColumns[14], c15=allColumns[15], c16=allColumns[16], c17=allColumns[17], c18=allColumns[18], c19=allColumns[19])
    end

    date = Dates.now()
    dateString=Dates.format(date, "yyyy-mm-dd_HH.MM.SS")
    if operationMode == "minimum running time"
        csvFilePath=settings[:csvDirectory]*"/"*dateString*"_MinimumRunningTime.csv"
    elseif operationMode == "minimum energy consumption"
        csvFilePath=settings[:csvDirectory]*"/"*dateString*"_MinimumEnergyConsumption.csv"
    else
        # should not be possible
    end
    CSV.write(csvFilePath, df, header=false)
    println("The output CSV file has been created for ",operationMode," at ",csvFilePath)

    return true
end #function createCsvFile

end #module Export
