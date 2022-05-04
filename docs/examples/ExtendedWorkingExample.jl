#!/usr/bin/env julia

import TrainRun

paths=[]
push!(paths, importFromYaml(:path, "data/paths/path_1_10km_nConst_vConst.yaml"))
push!(paths, importFromYaml(:path, "data/paths/path_2_10km_nVar_vConst.yaml"))
push!(paths, importFromYaml(:path, "data/paths/path_3_10km_nConst_vVar.yaml"))
push!(paths, importFromYaml(:path, "data/paths/path_4_real_Germany_EastSaxony_DG-DN.yaml"))

settings=[]
push!(settings, importFromYaml(:settings, "data/settings/settings_distanceStep_massPoint.yaml"))

trains=[]
push!(trains, importFromYaml(:train, "data/trains/train_freight_V90withOreConsist.yaml"))
push!(trains, importFromYaml(:train, "data/trains/train_passenger_SiemensDesiroClassic.yaml"))
push!(trains, importFromYaml(:train, "data/trains/train_passenger_IC2.yaml"))

for path in paths
    # println(" -    -    -     -     -     -      -     -    -")
    # println("path: ", path[:name])
   for train in trains
       # println("train: ", train[:name])
       for settings in settings
           resultsDict = trainrun(train, path, settings)
           if haskey(settings, :outputFormat) && settings[:outputFormat] == "CSV"
               exportToCsv(resultsDict, settings)
               sleep(2)
           end
       end
   end
end
