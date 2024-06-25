#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2024"
# __license__       = "ISC"

using TrainRuns, Test

include("logging_levels.jl")
include("df_2_vector.jl")
include("getNextPoiPosition.jl")
include("getOutputByDetail.jl")
include("highlevel_test.jl")
include("value_changes.jl")
