#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

using TrainRuns, Test

trains = Dict()
paths = Dict()
settings = Dict()

@testset "load data" begin

  println("testing load train data")
  push!(trains, :freight      => @time Train("data/trains/freight.yaml"))
  push!(trains, :local        => @time Train("data/trains/local.yaml"))
  push!(trains, :longdistance => @time Train("data/trains/longdistance.yaml"))

  println("testing load path data")
  push!(paths, :const     => @time Path("data/paths/const.yaml"))
  push!(paths, :slope     => @time Path("data/paths/slope.yaml"))
  push!(paths, :speed     => @time Path("data/paths/speed.yaml"))
  push!(paths, :realworld => @time Path("data/paths/realworld.yaml"))

  println("testing load settings data")
  push!(settings, "default"       => @time Settings())
  push!(settings, "poi"           => @time Settings("data/settings/points_of_interest.yaml"))
  push!(settings, "drivingcourse" => @time Settings("data/settings/driving_course.yaml"))
  push!(settings, "everything"    => @time Settings("data/settings/everything.yaml"))
  push!(settings, "strip"         => @time Settings("data/settings/strip.yaml"))
  push!(settings, "time"          => @time Settings("data/settings/time.yaml"))
  push!(settings, "timestrip"     => @time Settings("data/settings/time_strip.yaml"))
  push!(settings, "velocity"      => @time Settings("data/settings/velocity.yaml"))

  @test typeof(first(paths)[2]) == Path
  @test typeof(first(settings)[2]) == Settings

end

println("====================")

tests = Base.Iterators.product(trains,paths)

## routine to generate the anticipated Dict()
# anticipated = Dict()
# for test in tests
#   println(test[1][1],"-",test[2][1])
#   result = @time trainrun(test[1][2],test[2][2])
#   push!(anticipated, Symbol(String(test[1][1]) * "_" * String(test[2][1])) => result )
# end

anticipated = Dict(
  :default => Dict(
    :longdistance_speed => 499.96109564970516,
    :freight_slope => 831.4768274141168,
    :local_slope => 396.99313307033276,
    :longdistance_const => 328.83479381353095,
    :freight_realworld => 8971.50124080998,
    :longdistance_slope => 329.22915822053164,
    :freight_const => 727.7969403041934,
    :longdistance_realworld => 2900.1198723158523,
    :local_speed => 524.3948201513945,
    :local_realworld => 3443.917823618831,
    :freight_speed => 733.2610572579886,
    :local_const => 392.7234008268302
  )
)

@testset "function trainrun()" begin

  @testset "Default settings" begin

    for test in tests
      test_name = String(test[1][1]) * "_" * String(test[2][1])
      println("testing $test_name")
      @time result = trainrun(test[1][2],test[2][2])
      expected = anticipated[:default][Symbol(test_name)]
      # compare result to test data set
      @test isapprox(result, expected, rtol=0.01)
      println("--------------------")
    end

  end
  println("====================")

end
