#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg"
# __copyright__     = "2021"
# __license__       = "ISC"

include("../src/TrainRun.jl")
using .TrainRun

allPaths=[]
push!(allPaths, importYamlFile(:path, "data/paths/path_1_10km_nConst_vConst.yaml"))
push!(allPaths, importYamlFile(:path, "data/paths/path_2_10km_nVar_vConst.yaml"))
push!(allPaths, importYamlFile(:path, "data/paths/path_3_10km_nConst_vVar.yaml"))
push!(allPaths, importYamlFile(:path, "data/paths/path_4_real_Germany_EastSaxony_DG-DN.yaml"))

allSettings=[]
push!(allSettings, importYamlFile(:settings, "data/settings/settings_distanceStep_massPoint.yaml"))

allTrains=[]
push!(allTrains, importYamlFile(:train, "data/trains/train_freight_V90withOreConsist.yaml"))
push!(allTrains, importYamlFile(:train, "data/trains/train_passenger_SiemensDesiroClassic.yaml"))
push!(allTrains, importYamlFile(:train, "data/trains/train_passenger_IC2.yaml"))

for path in allPaths
    # println(" -    -    -     -     -     -      -     -    -")
    # println("path: ", path)
   for train in allTrains
       # println("train: ", train)
       for settings in allSettings
           resultsDict = calculateDrivingDynamics(train, path, settings)
           exportToCsv(resultsDict)
           sleep(2)
           # println("")
       end
   end
   # println("")
end

# println("")
# println("________________________")
# println("")
