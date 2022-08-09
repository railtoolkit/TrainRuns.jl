#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

function createOutput(settings::Settings, drivingCourse::Vector{Dict}, pointsOfInterest::Vector{Tuple})
    if settings.outputDetail == :running_time
        output::Vector{Dict} = [Dict(:t => drivingCourse[end][:t])]

    elseif settings.outputDetail == :points_of_interest
        # get only the driving course's support points with POI labels
        # if there is no point with POI label return the information of departure and arrival (first and last points)
        output = Dict[]
        if isempty(pointsOfInterest)
            push!(output, drivingCourse[1])
            push!(output, drivingCourse[end])
        else
            supportPoint = 1
            for POI in 1:length(pointsOfInterest)
                while supportPoint <= length(drivingCourse)
                    if pointsOfInterest[POI][1] == drivingCourse[supportPoint][:s]
                        push!(output, drivingCourse[supportPoint])
                        break
                    end
                    supportPoint += 1
                end
            end
        end

    elseif settings.outputDetail == :data_points
        # get the driving course's support points where a new behavior section starts and the driving mode changes
        output = Dict[]
        # the first support point is the first data point
        push!(output, drivingCourse[1])

        for supportPoint in 2:length(drivingCourse)
            if drivingCourse[supportPoint-1][:behavior] != drivingCourse[supportPoint][:behavior]
                push!(output, drivingCourse[supportPoint])
            end
        end

    elseif settings.outputDetail == :driving_course
        output = drivingCourse
    end

    if settings.outputFormat == :dataframe
        return createDataFrame(output, settings.outputDetail, settings.approxLevel)
    elseif settings.outputFormat == :vector
        return output
    end
end


function createDataFrame(output_vector::Vector{Dict}, outputDetail, approxLevel::Int)
    if outputDetail == :running_time
        # create a DataFrame with running time information
        dataFrame = DataFrame(t=[round(output_vector[end][:t], digits=approxLevel)])
    else # :points_of_interest, :data_points or :driving_course
        columnSymbols = [:label, :behavior, :s, :v, :t, :a, :F_T, :F_R, :R_path, :R_traction, :R_wagons]

        allColumns = []
        for column in 1:length(columnSymbols)
            if typeof(output_vector[1][columnSymbols[column]]) == String
                currentStringColumn::Vector{String} = []
                for point in output_vector
                    push!(currentStringColumn, point[columnSymbols[column]])
                end
                push!(allColumns, currentStringColumn)
            elseif typeof(output_vector[1][columnSymbols[column]]) <: Real
                currentRealColumn::Vector{Real} = []
                for point in output_vector
                    push!(currentRealColumn, point[columnSymbols[column]])
                end
                currentRealColumn = round.(currentRealColumn, digits=approxLevel)
                push!(allColumns, currentRealColumn)
            end
        end # for

        # combine the columns in a data frame
        dataFrame = DataFrame(label=allColumns[1], driving_mode=allColumns[2], s=allColumns[3], v=allColumns[4], t=allColumns[5], a=allColumns[6], F_T=allColumns[7], F_R=allColumns[8], R_path=allColumns[9], R_traction=allColumns[10], R_wagons=allColumns[11])
    end

    return dataFrame
end #createDataFrame
