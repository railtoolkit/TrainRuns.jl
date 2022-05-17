#!/usr/bin/env julia

using TrainRuns

paths=[]
push!(paths, Path("test/data/paths/const.yaml"))
push!(paths, Path("test/data/paths/slope.yaml"))
push!(paths, Path("test/data/paths/speed.yaml"))
push!(paths, Path("test/data/paths/realworld.yaml"))

settings=[]
push!(settings, Settings("test/data/settings/driving_course.yaml"))

trains=[]
push!(trains, Train("test/data/trains/freight.yaml"))
push!(trains, Train("test/data/trains/local.yaml"))
push!(trains, Train("test/data/trains/longdistance.yaml"))

driving_courses=[]
for path in paths
    # println(" -    -    -     -     -     -      -     -    -")
    # println("path: ", path[:name])
   for train in trains
       # println("train: ", train[:name])
       for settings in settings
           push!(driving_courses, trainrun(train, path, settings))
           #driving_course = trainrun(train, path, settings)

    # old:      if haskey(settings, :outputFormat) && settings[:outputFormat] == "CSV"
    # old:          exportToCsv(resultsDict, settings)
    # old:          sleep(2)
    # old:      end
       end
   end
end
