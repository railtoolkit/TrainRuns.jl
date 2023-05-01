#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

"""
    createOutput(settings, drivingCourse, pointsOfInterest)

Create output information depending on `settings`, `drivingCourse` and `pointsOfInterest`.

See also [`createOutput`](@ref).

# Arguments
- `settings::Settings`: the Settings object containing settings for output format and detail.
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `pointsOfInterest::Vector{NamedTuple}`: the Vector containing tuples for the paths' points of interest.

# Examples
```julia-repl
julia> createOutput(settings_poi, drivingCourse_longdistance, pointsOfInterest_pathWithSlope)
5×11 DataFrame
 Row │ label             driving_mode  s        v       t        a       F_T        F_R      R_path   R_traction  R_wagons
     │ String            String        Real     Real    Real     Real    Real       Real     Real     Real        Real
─────┼─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   1 │ view_point_1      accelerating   850.0   28.707   54.049   0.331  1.93049e5  36602.1     0.0      9088.56   27513.6
   2 │ distant_signal_1  accelerating  1000.0   30.325   59.129   0.294  1.82746e5  43604.7  4344.35     9795.13   29465.2
   3 │ main_signal_1     accelerating  2000.0   37.356   88.468   0.185  1.48352e5  60899.4  8688.69    13259.1    38951.5
   4 │ main_signal_3     braking       9000.0   27.386  258.578  -0.375  0.0        34522.1     0.0      8537.05   25985.0
   5 │ clearing_point_1  braking       9203.37  24.443  266.426  -0.375  0.0        30176.2     0.0      7389.44   22786.8
```
"""
function createOutput(settings::Settings, drivingCourse::Vector{Dict}, poi_positions::Vector{Any})
    if settings.outputDetail == :running_time
        output::Vector{Dict} = [Dict(:t => drivingCourse[end][:t])]

    elseif settings.outputDetail == :points_of_interest
        # get only the driving course's support points with POI labels
        # if there is no point with POI label return the information of departure and arrival (first and last points)
        output = Dict[]
        if isempty(poi_positions)
            push!(output, drivingCourse[1])
            push!(output, drivingCourse[end])
        else
            for supportPoint in drivingCourse
                if supportPoint[:s] in poi_positions
                    push!(output, supportPoint)
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


"""
    createDataFrame(output_vector, outputDetail, approxLevel)

Create a DataFrame from `output_vector` with `outputDetail` and `approxLevel`.

See also [`createOutput`](@ref).

# Arguments

- `output_vector::Vector{Dict}`: the Vector containing all data to be outputted.
- `outputDetail::Symbol`: the detail level the DataFrame is created for.
- `approxLevel::Int`: the number of digits for rounding each Number in the DataFrame.

# Examples
```julia-repl
julia> createDataFrame(vector_pointsOfInterest, detail_data_points, approxLevel_default)
5×11 DataFrame
 Row │ label             driving_mode  s        v       t        a       F_T        F_R      R_path   R_traction  R_wagons
     │ String            String        Real     Real    Real     Real    Real       Real     Real     Real        Real
─────┼─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   1 │ view_point_1      accelerating   850.0   28.707   54.049   0.331  1.93049e5  36602.1     0.0      9088.56   27513.6
   2 │ distant_signal_1  accelerating  1000.0   30.325   59.129   0.294  1.82746e5  43604.7  4344.35     9795.13   29465.2
   3 │ main_signal_1     accelerating  2000.0   37.356   88.468   0.185  1.48352e5  60899.4  8688.69    13259.1    38951.5
   4 │ main_signal_3     braking       9000.0   27.386  258.578  -0.375  0.0        34522.1     0.0      8537.05   25985.0
   5 │ clearing_point_1  braking       9203.37  24.443  266.426  -0.375  0.0        30176.2     0.0      7389.44   22786.8
```
"""
function createDataFrame(output_vector::Vector{Dict}, outputDetail::Symbol, approxLevel::Int)
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
