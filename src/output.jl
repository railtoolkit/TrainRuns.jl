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
function createOutput(
        settings::Settings,
        drivingCourse::Vector{Dict},
        pois::DataFrame
)
    drivingCourse = DataFrame(drivingCourse)

    output = getOutputByDetail(drivingCourse, pois, settings.outputDetail)

    if settings.outputFormat == :vector
        return df_2_vector(output)
    end

    return output
end

"""
    getOutputByDetail(drivingCourse::DataFrame, pois::DataFrame, outputDetail::Symbol)::DataFrame

Filter drivingCourse depending on output detail.

`outputDetail` can be `:running_time`, `:points_of_interest`, `:data_points` or `:driving_course`
"""
function getOutputByDetail(
        drivingCourse::DataFrame, pois::DataFrame, outputDetail::Symbol)::DataFrame
    if outputDetail == :running_time
        return DataFrame(t = drivingCourse[end, :t])
    elseif outputDetail == :points_of_interest
        if size(pois, 1) == 0
            output = drivingCourse[[1, end], :]
            output.label = ["", ""]
            return output
        end

        return rightjoin(drivingCourse, pois, on = :s, order = :left)
    elseif outputDetail == :data_points || outputDetail == :driving_course
        output = leftjoin(drivingCourse, pois[:, [:s, :label]], on = :s, order = :left)
        replace!(output.label, missing => "")

        if outputDetail == :data_points
            subset!(output, [:behavior] => value_changes)
        end

        return output
    end
end

function get_loglevel(settings::Settings)::LogLevel
    current_logger = global_logger()
    if settings.verbosity != :unset
        current_level = lowercase(String(Symbol(current_logger.min_level)))
        new_level = String(settings.verbosity)
        if current_level != new_level
            @debug "Changing log level from `$current_level` to `$new_level`."
            if settings.verbosity == :trace
                loglevel = Trace
            elseif settings.verbosity == :debug
                loglevel = Logging.Debug
            elseif settings.verbosity == :info
                loglevel = Logging.Info
            elseif settings.verbosity == :warn
                loglevel = Logging.Warn
            elseif settings.verbosity == :error
                loglevel = Logging.Error
            elseif settings.verbosity == :fatal
                loglevel = Fatal
            else
                @warn "Did not recognize the log level `$new_level`." "Log level was not changed!"
            end
        else
            loglevel = current_logger.min_level
        end
    else
        loglevel = current_logger.min_level
    end
    return loglevel
end
