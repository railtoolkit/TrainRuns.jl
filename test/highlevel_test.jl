#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.0
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2021"
# __license__       = "ISC"

trains = Dict()
paths = Dict()
settings = Dict()

@testset "load data" begin
    @info "testing load train data"
    push!(trains, :freight => @time Train("data/trains/freight.yaml"))
    push!(trains, :local => @time Train("data/trains/local.yaml"))
    push!(trains, :longdistance => @time Train("data/trains/longdistance.yaml"))

    @info "testing load path data"
    push!(paths, :const => @time Path("data/paths/const.yaml"))
    push!(paths, :slope => @time Path("data/paths/slope.yaml"))
    push!(paths, :speed => @time Path("data/paths/speed.yaml"))
    push!(paths, :realworld => @time Path("data/paths/realworld.yaml"))

    @info "testing load settings data"
    push!(settings, "default" => @time Settings())
    push!(settings, "poi" => @time Settings("data/settings/points_of_interest.yaml"))
    push!(settings, "drivingcourse" => @time Settings("data/settings/driving_course.yaml"))
    push!(settings, "strip" => @time Settings("data/settings/strip.yaml"))
    push!(settings, "time" => @time Settings("data/settings/time.yaml"))
    push!(settings, "timestrip" => @time Settings("data/settings/time_strip.yaml"))
    push!(settings, "velocity" => @time Settings("data/settings/velocity.yaml"))

    @test typeof(first(paths)[2]) == Path
    @test typeof(first(settings)[2]) == Settings

    @test_throws DomainError Path("data/paths/broken.yaml")
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

@testset "function trainrun()" begin
    @testset "Default settings" begin
        for test in tests
            test_name = String(test[1][1]) * "_" * String(test[2][1])
            @info "testing $test_name"
            @time result = trainrun(test[1][2], test[2][2])[end, :t]
            expected = anticipated[:default][Symbol(test_name)]
            # compare result to test data set
            @test isapprox(result, expected, rtol = 0.01)
        end
    end
end
