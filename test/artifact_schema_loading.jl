#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2024"
# __license__       = "ISC"

using LazyArtifacts, YAML, JSONSchema

@testset "artifact schema loading" begin
    railtoolkit_schema_2022_running_path = Dict(
        "schema_version" => "2022.05",
        "schema" => "https://railtoolkit.org/schema/running-path.json"
    )
    railtoolkit_schema_2022_rolling_stock = Dict(
        "schema_version" => "2022.05",
        "schema" => "https://railtoolkit.org/schema/rolling-stock.json"
    )

    artifactpath_railtoolkit_schema_2022 = artifact"railtoolkit-org-2022-05"
    artifactpath_sha1 = split(artifactpath_railtoolkit_schema_2022, "/")[end]
    @test artifactpath_sha1 == "0dc2382451d64a12b8e8aa95c83a81dc5ac1c108"

    railtoolkit_schema_2022_running_path = TrainRuns.get_schema(railtoolkit_schema_2022_running_path)
    path_schema_2022 = YAML.load(open("data/paths/path_schema_2022.yaml"))
    @test JSONSchema.isvalid(railtoolkit_schema_2022_running_path, path_schema_2022)

    railtoolkit_schema_2022_rolling_stock = TrainRuns.get_schema(railtoolkit_schema_2022_rolling_stock)
    train_schema_2022 = YAML.load(open("data/trains/train_schema_2022.yaml"))
    @test JSONSchema.isvalid(railtoolkit_schema_2022_rolling_stock, train_schema_2022)

    unrecognized_schema = Dict(
        "schema_version" => "1",
        "schema" => "http://unrecognized-schema/subtype"
    )
    logger = ConsoleLogger(stderr, Warn)
    with_logger(logger) do
        @test_throws ErrorException("The provided schema 'http://unrecognized-schema/subtype' version '1' is not recognized!") TrainRuns.get_schema(unrecognized_schema)
    end
end
