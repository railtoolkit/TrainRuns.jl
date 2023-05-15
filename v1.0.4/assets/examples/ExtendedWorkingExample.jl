#!/usr/bin/env julia

using TrainRuns
using CSV

paths=[]
push!(paths, Path("test/data/paths/const.yaml"))
push!(paths, Path("test/data/paths/slope.yaml"))
push!(paths, Path("test/data/paths/speed.yaml"))
push!(paths, Path("test/data/paths/realworld.yaml"))

trains=[]
push!(trains, Train("test/data/trains/freight.yaml"))
push!(trains, Train("test/data/trains/local.yaml"))
push!(trains, Train("test/data/trains/longdistance.yaml"))

settings = Settings("test/data/settings/driving_course.yaml")

for p in 1:length(paths)
   for t in 1:length(trains)
       driving_course = trainrun(trains[t], paths[p], settings)
       CSV.write("docs/examples/drivingCourse_path"*string(p)*"_train"*string(t)*".csv", driving_course)
   end
end
