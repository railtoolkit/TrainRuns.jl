#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg"
# __copyright__     = "2021"
# __license__       = "ISC"

include("../src/TrainRun.jl")
using .TrainRun

allPaths=[]
push!(allPaths, importFromYaml(:path, "data/paths/path_1_10km_nConst_vConst.yaml"))
push!(allPaths, importFromYaml(:path, "data/paths/path_2_10km_nVar_vConst.yaml"))
push!(allPaths, importFromYaml(:path, "data/paths/path_3_10km_nConst_vVar.yaml"))
push!(allPaths, importFromYaml(:path, "data/paths/path_4_real_Germany_EastSaxony_DG-DN.yaml"))

allSettings=[]
push!(allSettings, importFromYaml(:settings, "data/settings/settings_distanceStep_massPoint.yaml"))

allTrains=[]
push!(allTrains, importFromYaml(:train, "data/trains/train_freight_V90withOreConsist.yaml"))
push!(allTrains, importFromYaml(:train, "data/trains/train_passenger_SiemensDesiroClassic.yaml"))
push!(allTrains, importFromYaml(:train, "data/trains/train_passenger_IC2.yaml"))

for path in allPaths
    # println(" -    -    -     -     -     -      -     -    -")
    # println("path: ", path[:name])
   for train in allTrains
       # println("train: ", train[:name])
       for settings in allSettings
           resultsDict = calculateDrivingDynamics(train, path, settings)
           if haskey(settings, :typeOfOutput) && settings[:typeOfOutput] == "CSV"
               exportToCsv(resultsDict, settings)
               sleep(2)
           end
           # println("")
       end
   end
   # println("")
end

# println("")
# println("________________________")
# println("")
