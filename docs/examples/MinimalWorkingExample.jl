#!/usr/bin/env julia

import TrainRun

train = Train("data/trains/train_freight_V90withOreConsist.yaml")
path  = Path("data/paths/path_1_10km_nConst_vConst.yaml")

runtime = trainRun(train, path)

println("The train needs $runtime seconds for the running path.")
