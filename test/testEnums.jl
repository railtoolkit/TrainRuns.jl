#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

include("../src/types.jl")
include("../src/Input.jl")

using .Input
using YAML, Test

@enum trainTypes passenger=1 freight=2 motorCoachTrain=3

@test Input.getEnum("passenger", trainTypes)       == passenger::trainTypes
@test Input.getEnum("freight", trainTypes)         == freight::trainTypes
@test Input.getEnum("motorCoachTrain", trainTypes) == motorCoachTrain::trainTypes

data = YAML.load(open("data/trains/train_passenger_IC2.yaml"))
@test Input.getEnum(data["train"]["trainType"], trainTypes) == passenger::trainTypes

data = YAML.load(open("data/trains/train_freight_V90withOreConsist.yaml"))
@test Input.getEnum(data["train"]["trainType"], trainTypes) == freight::trainTypes