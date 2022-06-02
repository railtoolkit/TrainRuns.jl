#!/usr/bin/env julia

using TrainRuns

train = Train("test/data/trains/freight.yaml")
path  = Path("test/data/paths/const.yaml")

runtime_dataFrame = trainrun(train, path)
runtime = runtime_dataFrame[!, 1][2]

println("The train needs $runtime seconds for the running path.")
