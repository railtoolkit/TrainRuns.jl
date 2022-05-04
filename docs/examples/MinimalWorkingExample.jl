#!/usr/bin/env julia

using TrainRun

train = Train("test/data/trains/freight.yaml")
path  = Path("test/data/paths/const.yaml")

runtime = trainrun(train, path)

println("The train needs $runtime seconds for the running path.")
