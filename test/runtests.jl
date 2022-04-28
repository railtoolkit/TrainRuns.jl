#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

using TrainRun, Test

paths=Dict()
push!(paths, "const"     => TrainRun.importFromYaml(:path, "test/data/paths/const.yaml"))
push!(paths, "slope"     => TrainRun.importFromYaml(:path, "test/data/paths/slope.yaml"))
push!(paths, "speed"     => TrainRun.importFromYaml(:path, "test/data/paths/speed.yaml"))
push!(paths, "realworld" => TrainRun.importFromYaml(:path, "test/data/paths/realworld.yaml"))

settings=Dict()
push!(settings, "default"        => Settings())
push!(settings, "detail"         => Settings("test/data/settings/detail.yaml"))
push!(settings, "driving_course" => Settings("test/data/settings/driving_course.yaml"))
push!(settings, "strip"          => Settings("test/data/settings/strip.yaml"))
push!(settings, "time"           => Settings("test/data/settings/time.yaml"))
push!(settings, "time_strip"     => Settings("test/data/settings/time_strip.yaml"))
push!(settings, "velocity"       => Settings("test/data/settings/velocity.yaml"))
push!(settings, "csv_export"     => Settings("test/data/settings/csv_export.yaml"))

trains=Dict()
push!(trains, TrainRun.importFromYaml(:train, "test/data/trains/freight.yaml"))
push!(trains, TrainRun.importFromYaml(:train, "test/data/trains/local.yaml"))
push!(trains, TrainRun.importFromYaml(:train, "test/data/trains/longdistance.yaml"))

@testset "TrainRun.jl" begin

  @testset "Default settings" begin

    @test typeof(Settings()) == Settings

    @testset "const path" begin

      path = TrainRun.importFromYaml(:path, "test/data/paths/const.yaml")
      @test typeof(path) == Dict{Any,Any}

      @testset "freight train - const path" begin
        train = TrainRun.importFromYaml(:train, "test/data/trains/freight.yaml")
        data = trainRun(train, path)
        expected = 727.796900196972
        # compare result to test data set
        @test isapprox(data, expected, atol=0.01)
      end

      @testset "local train - const path" begin
        train = TrainRun.importFromYaml(:train, "test/data/trains/local.yaml")
        data = trainRun(train, path)
        expected = 392.723361763612
        # compare result to test data set
        @test isapprox(data, expected, atol=0.01)
      end

      @testset "long distance train - const path" begin
        train = TrainRun.importFromYaml(:train, "test/data/trains/longdistance.yaml")
        data = trainRun(train, path)
        expected = 328.83487704779117
        # compare result to test data set
        @test isapprox(data, expected, atol=0.01)
      end

    end
  end

end
