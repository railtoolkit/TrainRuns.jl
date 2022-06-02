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
        # create DataFrame with running time information
        dataFrame = DataFrame(column1=["t (in s)", output_vector[end][:t]])
    else # :points_of_interest or :driving_course
        header = ["label", "driving mode", "s (in m)", "v (in m/s)", "t (in s)", "a (in m/s^2)", "F_T (in N)", "F_R (in N)", "R_path (in N)", "R_traction (in N)", "R_wagons (in N)"]
        columnSymbols = [:label, :behavior, :s, :v, :t, :a, :F_T, :F_R, :R_path, :R_traction, :R_wagons]

        allColumns = Array{Any,1}[]
        for column in 1:length(header)
            currentColumn = Any[]
            push!(currentColumn, header[column])
            for point in output_vector
                push!(currentColumn, point[columnSymbols[column]])
            end
            push!(allColumns, currentColumn)
        end # for

        # combine the columns in a data frame
        dataFrame = DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3], c4=allColumns[4], c5=allColumns[5], c6=allColumns[6], c7=allColumns[7], c8=allColumns[8], c9=allColumns[9], c10=allColumns[10], c11=allColumns[11])
    end

    return dataFrame
end #createDataFrameForDrivingCourse
