#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2020-2022"
# __license__       = "ISC"
__precompile__(true)

module TrainRuns

## loading standard library packages
using UUIDs, Dates, Statistics, Logging
## loading external packages
using YAML, JSON, JSONSchema, DataFrames, LoggingExtras

## Interface
export trainrun, Train, Path, Settings

## global variables
global g = 9.80665         # acceleration due to gravity (in m/s^2)
global μ = 0.2             # friction as constant, TODO: implement as function
global Δv_air = 15.0 / 3.6 # coefficient for velocitiy difference between train and outdoor air (in m/s)

const Trace = Logging.LogLevel(-2000)
const Fatal = Logging.LogLevel(3000)

## include package files
include("types.jl")
include("constructors.jl")
include("formulary.jl")
include("calc.jl")
include("behavior.jl")
include("output.jl")
include("utils.jl")

## main function
"""
    trainrun(train::Train, path::Path[, settings::Settings])

Calculate the running time of a [`Train`](@ref) on a [`Path`](@ref) in seconds.

See also [`Settings`](@ref).

# Examples
```julia-repl
julia> trainrun(train, path)[end,:t]
xxx.xx # in seconds
```
"""
function trainrun(train::Train, path::Path, settings = Settings()::Settings)
    loglevel = get_loglevel(settings)
    logger = LoggingExtras.LevelOverrideLogger(loglevel, global_logger()) # Bug - Log messages below LogLevel(-1000) are ignored
    # workaround:
    Logging.disable_logging(loglevel - 1) # https://github.com/JuliaLang/julia/issues/52234

    with_logger(logger) do
        @debug "" train
        @debug "" path
        @debug "" settings

        # prepare the input data
        (characteristicSections, pois) = determineCharacteristics(
            path, train, settings)

        # calculate the train run with the minimum running time
        drivingCourse = calculateMinimumRunningTime(characteristicSections, settings, train)

        # accumulate data and create an output dictionary
        output = createOutput(settings, drivingCourse, pois)

        return output
    end
end # function trainrun

"""
    trainrun(path::Path, train::Train[, settings::Settings])

Alias of [`trainrun(train::Train, path::Path)`](@ref) with swapped arguments.
"""
function trainrun(path::Path, train::Train, settings = Settings()::Settings)
    trainrun(train, path, settings)
end # function trainrun alias

end # module TrainRuns
