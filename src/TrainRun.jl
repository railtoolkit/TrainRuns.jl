#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2020-2022"
# __license__       = "ISC"
__precompile__(true)

module TrainRun

## loading standard library packages
using UUIDs, Dates
## loading external packages
using YAML, JSONSchema, CSV, DataFrames

export 
## Interface
trainrun, Path, Settings, exportToCsv

## include package files
include("types.jl")
include("formulary.jl")
include("characteristics.jl")
include("behavior.jl")
include("output.jl")
include("import.jl")
include("export.jl")
include("calc.jl")

## main function
"""
    trainrun(train::Dict, path::Path, settings::Settings)

Calculate the running time of a `train` on a `path`.
The `settings` provides the choice of models for the calculation.
`settings` can be omitted. If so, a default is used.
The running time will be return in seconds.

# Examples
```julia-repl
julia> trainrun(train, path)
xxx.xx # in seconds
```
"""
function trainrun(trainInput::Dict, path::Path, settings=Settings()::Settings)
    # copy Input data for not changing them
    # TODO: or should they be changed? normally it would only make it "better" except for settings.outputDetail == :points_of_interest && isempty(path.poi)
    train = copy(trainInput)

    # check the input data
    train = checkAndSetTrain!(train)
    settings.outputDetail == :everything && println("The input has been checked.")

    # prepare the input data
    movingSection = determineCharacteristics(path, train, settings)
    settings.outputDetail == :everything && println("The moving section has been prepared.")

    # calculate the train run for oparation mode "minimum running time"
    (movingSection, drivingCourse) = calculateMinimumRunningTime!(movingSection, settings, train)
    settings.outputDetail == :everything && println("The driving course for the shortest running time has been calculated.")

    # accumulate data and create an output dictionary
    output = createOutput(train, settings, path, movingSection, drivingCourse)

    return output
end # function trainrun

end # module TrainRun
