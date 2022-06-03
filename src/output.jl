#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

function createOutput(settings::Settings, path::Path, drivingCourse::Vector{Dict})
    if settings.outputDetail == :running_time
        output::Vector{Dict} = [Dict(:t => drivingCourse[end][:t])]

    elseif settings.outputDetail == :points_of_interest && !isempty(path.poi)
        # add points of interest

        #    output = Dict[]
        #    POI = 1
        #    i = 1
        #    while POI <= length(path.poi) && i <= drivingCourse[end][:i]
        #        if path.poi[POI][:station] == drivingCourse[i][:s]
        #            push!(output, drivingCourse[i])
        #            POI = POI+1
        #        end
        #        i = i+1
        #    end

        # get only the driving course's data points with POI labels
        output = Dict[]
        for point in drivingCourse
            if point[:label] != ""
                push!(output, point)
            end
        end

    else #if settings.outputDetail == :driving_course || (settings.outputDetail == :points_of_interest && !isempty(path.poi))
            output = drivingCourse
    end

    if settings.outputFormat == :dataframe
        return createDataFrame(output, settings.outputDetail)
    elseif settings.outputFormat == :vector
        return output
    end
end


function createDataFrame(output_vector::Vector{Dict}, outputDetail)
    if outputDetail == :running_time
        # create a DataFrame with running time information
        dataFrame = DataFrame(t=[output_vector[end][:t]])
    else # :points_of_interest or :driving_course
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
                push!(allColumns, currentRealColumn)
            end
        end # for

        # combine the columns in a data frame
        dataFrame = DataFrame(label=allColumns[1], driving_mode=allColumns[2], s=allColumns[3], v=allColumns[4], t=allColumns[5], a=allColumns[6], F_T=allColumns[7], F_R=allColumns[8], R_path=allColumns[9], R_traction=allColumns[10], R_wagons=allColumns[11])
    end

    return dataFrame
end #createDataFrameForDrivingCourse
