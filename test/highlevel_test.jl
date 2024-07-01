#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

using CSV, DataFrames

"""
    @df_snapshot_test(name, expr::Expr)

Saves DataFrames to CSV and compares them.
To renew snapshot delete the file named like the test.
"""
macro df_snapshot_test(dir, name, expr::Expr)
    return quote
        filename = $(esc(name)) * ".csv"
        filepath = joinpath(Base.source_dir(), "snapshots", $(dir), filename)
        df = $(esc(expr))

        if !isfile(filepath)
            if !isdir(dirname(filepath))
                mkdir(dirname(filepath))
            end

            CSV.write(filepath, df, missingstring = "!missing!")
            @warn "Snapshot file $filename created"
        end

        csv_df = DataFrame(CSV.File(filepath; missingstring = "!missing!"))

        @test df == csv_df
    end
end

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

## routine to generate the anticipated Dict()
# anticipated = Dict()
# for test in tests
#   @info test[1][1],"-",test[2][1]
#   result = @time trainrun(test[1][2],test[2][2])
#   push!(anticipated, Symbol(String(test[1][1]) * "_" * String(test[2][1])) => result )
# end

anticipated = Dict(
    :default => Dict(
    :freight_const => 746.6594691660882,
    :freight_slope => 842.3797947097586,
    :freight_speed => 751.9727357301351,
    :freight_realworld => 8789.299477891092,
    :local_const => 392.6801497584646,
    :local_slope => 396.59291844946534,
    :local_speed => 524.3734475900396,
    :local_realworld => 3438.3543735577446,
    :longdistance_const => 330.8511578156266,
    :longdistance_slope => 331.7163794230447,
    :longdistance_speed => 501.13448446081713,
    :longdistance_realworld => 2913.3759609192407
),
)

@testset "trainrun()" begin
    @testset "Default settings" begin
        for test in tests
            train = test[1]
            path = test[2]
            test_name::String = String(train[1]) * "_" * String(path[1])

            @testset "$test_name" begin
                result = trainrun(test[1][2], test[2][2])[end, :t]
                expected = anticipated[:default][Symbol(test_name)]
                # compare result to test data set
                @test isapprox(result, expected, rtol = 0.01)
            end
        end
    end

    @testset "POIs" begin
        for test in tests
            train = test[1]
            path = test[2]
            test_name::String = String(train[1]) * "_" * String(path[1])

            @testset "$test_name" begin
                @df_snapshot_test "poi" "$test_name" trainrun(
                    train[2], path[2], settings["poi"])
            end
        end
    end
end
