#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

using DataFrames

function createOutput(train::Train, settings::Settings, path::Path, movingSection::Dict, drivingCourse::Vector{Dict})
    if settings.outputDetail == :running_time
        output = drivingCourse[end][:t]

    elseif settings.outputDetail == :points_of_interest
        # add points of interest
        if !isempty(path.poi)

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
            for point in 1:length(drivingCourse)
                if point[:label] != ""
                    push!(output, point)
                end
            end
        end

    elseif settings.outputDetail == :driving_course
            output = drivingCourse

    elseif settings.outputDetail == :everything
        output = Dict{Symbol,Any}()
        merge!(output, Dict(:train => train, :path => path, :settings => settings))


        # add moving section and driving courses
        merge!(output, Dict(:movingSection => movingSection,
                            :drivingCourse => drivingCourse))


        # add points of interest
        if !isempty(path.poi)
            pointsOfInterest = Dict[]
            # get only the driving course's data points with POI labels
            output = Dict[]
            for point in 1:length(drivingCourse)
                if point[:label] != ""
                    push!(pointsOfInterest, point)
                end
            end

            merge!(output, Dict(:pointsOfInterest => pointsOfInterest))

        end

        if settings.outputFormat == :dataframe
            return createDataFrameForDataPoints(output[:drivingCourse])
        else
            return output
        end
    end

    if settings.outputFormat == :dataframe
        return createDataFrame(output)
    else
        return output
    end
end


function createDataFrame(runningTime::AbstractFloat)
    # create DataFrame with running time information
    dataFrame = DataFrame(column1=["t (in s)", runningTime])
end

function createDataFrame(dataPoints::Vector{Dict})
    header = ["i", "behavior", "station label", "Δs (in m)", "s (in m)", "Δt (in s)","t (in s)","Δv (in m/s)","v (in m/s)","F_T (in N)","F_R (in N)","R_path (in N)","R_train (in N)","R_traction (in N)","R_wagons (in N)","a (in m/s^2)"]
    columnSymbols = [:i, :behavior, :label, :Δs, :s, :Δt, :t, :Δv, :v, :F_T, :F_R, :R_path, :R_train, :R_traction, :R_wagons, :a]

    allColumns = Array{Any,1}[]
    for column in 1:length(header)
        currentColumn = Any[]
        push!(currentColumn, header[column])
        for point in dataPoints
            push!(currentColumn, point[columnSymbols[column]])
        end
        push!(allColumns, currentColumn)
    end # for

    # combine the columns in a data frame
    dataFrame = DataFrame(c1=allColumns[1], c2=allColumns[2],c3=allColumns[3], c4=allColumns[4], c5=allColumns[5], c6=allColumns[6], c7=allColumns[7], c8=allColumns[8], c9=allColumns[9], c10=allColumns[10], c11=allColumns[11], c12=allColumns[12], c13=allColumns[13], c14=allColumns[14], c15=allColumns[15], c16=allColumns[16])

    return dataFrame
end #createDataFrameForDrivingCourse
