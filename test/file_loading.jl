#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2024"
# __license__       = "ISC"

@testset "file loading" begin

    # file_extension
    @test typeof(TrainRuns.load("data/trains/train_schema_2022.json")) == Dict{String, Any}
    @test typeof(TrainRuns.load("data/trains/train_schema_2022.yaml")) == Dict{Any, Any}
    @test typeof(TrainRuns.load("data/paths/path_schema_2022.json")) == Dict{String, Any}
    @test typeof(TrainRuns.load("data/paths/path_schema_2022.yaml")) == Dict{Any, Any}

    logger = ConsoleLogger(stderr, Warn)
    with_logger(logger) do
        @test_throws ErrorException("Can not load 'data/paths/missing_schema.yml'. No attribute with 'schema' found!") TrainRuns.load("data/paths/missing_schema.yml")
        @test_throws ErrorException("Can not load 'data/paths/missing_schema_version.yml'. No attribute with 'schema_version' found!") TrainRuns.load("data/paths/missing_schema_version.yml")
        @test_throws ErrorException("Can not load 'data/paths/broken.xml'. Unsupported file extension!") TrainRuns.load("data/paths/broken.xml")
        @test_throws ErrorException("Can not load 'data/paths/broken.yaml'. Format not recognized!") TrainRuns.load("data/paths/broken.yaml")
        @test_throws ErrorException("Can not load 'data/paths/missing_path.yml'. Format not recognized!") TrainRuns.load("data/paths/missing_path.yml")
    end
end
# 