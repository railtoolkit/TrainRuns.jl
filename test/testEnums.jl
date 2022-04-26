#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

include("../src/types.jl")
include("../src/Validate.jl")

using .Input
using YAML, Test

@enum trainType passenger=1 freight=2 motorCoachTrain=3

@test Input.getEnum("passenger", trainType)       == passenger::trainType
@test Input.getEnum("freight", trainType)         == freight::trainType
@test Input.getEnum("motorCoachTrain", trainType) == motorCoachTrain::trainType

data = YAML.load(open("data/trains/train_passenger_IC2.yaml"))
@test Input.getEnum(data["train"]["trainType"], trainType) == passenger::trainType

data = YAML.load(open("data/trains/train_freight_V90withOreConsist.yaml"))
@test Input.getEnum(data["train"]["trainType"], trainType) == freight::trainType
