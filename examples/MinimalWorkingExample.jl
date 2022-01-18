#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg"
# __copyright__     = "2021"
# __license__       = "ISC"

include("../src/TrainRun.jl")
using .TrainRun

train_directory =  "data/trains/train_freight_V90withOreConsist.yaml"
running_path_directory = "data/paths/path_1_10km_nConst_vConst.yaml"
setting_directory = "data/settings.yaml"
(train, running_path, settings) = importYamlFiles(train_directory, running_path_directory, setting_directory)

train_run = calculateDrivingDynamics(train, running_path, settings)
runtime = last(train_run[:outputArrayMinimumRunningTime])[5]

println("The V 90 with 10 ore wagons needs $runtime seconds for 10 km with no gradient.")
