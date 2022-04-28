#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2020-2022"
# __license__       = "ISC"
__precompile__(true)

module TrainRun

## loading external packages
using YAML, JSONSchema, CSV, DataFrames, Dates

## include package files
include("types.jl")
include("formulary.jl")
include("characteristics.jl")
include("behavior.jl")
include("output.jl")
include("import.jl")
include("export.jl")
include("calc.jl")

export 
## Interface
trainRun, Settings, exportToCsv

## main functions

end # module TrainRun
