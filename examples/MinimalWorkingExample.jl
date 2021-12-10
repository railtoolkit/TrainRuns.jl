#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg"
# __copyright__     = "2021"
# __license__       = "ISC"

include("../src/TrainRun.jl")
using .TrainRun

train =  "data/trains/train_freight_V90withOreConsist.yaml"
running_path = "data/paths/path_1_10km_nConst_vConst.yaml"
setting = "data/settings.yaml"

train_run = calculateDrivingDynamics(train, running_path, setting)
runtime = last(train_run["outputArrayMinimumRunningTime"])[5]

println("The V 90 with 10 ore wagons needs $runtime seconds for 10 km with no gradient.")