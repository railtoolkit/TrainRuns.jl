#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

using CSV, DataFrames

trains = Dict()
paths = Dict()
settings = Dict()

@testset "load data" begin
    @testset "train" begin
        push!(trains, :freight => Train("data/trains/freight.yaml"))
        push!(trains, :local => Train("data/trains/local.yaml"))
        push!(trains, :longdistance => Train("data/trains/longdistance.yaml"))
    end

    @testset "path" begin
        push!(paths, :const => Path("data/paths/const.yaml"))
        push!(paths, :slope => Path("data/paths/slope.yaml"))
        push!(paths, :speed => Path("data/paths/speed.yaml"))
        push!(paths, :realworld => Path("data/paths/realworld.yaml"))
        @test_throws DomainError Path("data/paths/broken.yaml")
        @test typeof(first(paths)[2]) == Path
    end

    @testset "settings" begin
        push!(settings, "default" => Settings())
        push!(settings, "poi" => Settings("data/settings/points_of_interest.yaml"))
        push!(settings, "drivingcourse" => Settings("data/settings/driving_course.yaml"))
        push!(settings, "strip" => Settings("data/settings/strip.yaml"))
        push!(settings, "time" => Settings("data/settings/time.yaml"))
        push!(settings, "timestrip" => Settings("data/settings/time_strip.yaml"))
        push!(settings, "velocity" => Settings("data/settings/velocity.yaml"))
        @test typeof(first(settings)[2]) == Settings
    end
end

tests = Base.Iterators.product(trains, paths)

@testset "trainrun()" begin
    @testset "Default settings" begin
        for test in tests
            train = test[1]
            path = test[2]
            test_name::String = String(train[1]) * "_" * String(path[1])

            @testset "$test_name" begin
                @df_snapshot_test "default/$test_name" trainrun(
                    train[2], path[2])
            end
        end
    end

    @testset "POIs settings" begin
        for test in tests
            train = test[1]
            path = test[2]
            test_name::String = String(train[1]) * "_" * String(path[1])

            @testset "$test_name" begin
                @df_snapshot_test "poi/$test_name" trainrun(
                    train[2], path[2], settings["poi"])
            end
        end
    end
end
