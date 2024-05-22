#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2024"
# __license__       = "ISC"

using Logging

function log_messages()
    @debug "debug"
    @info  "info"
    @warn  "warn"
    @error "error"
end

@testset "logging levels" begin
    @test_logs min_level=Logging.Warn TrainRuns.set_log_level(Settings())
    @test_logs (:info,  "info") (:warn,  "warn") (:error, "error") min_level=Logging.Info log_messages()
    #
    @test_logs (:info, "Changing log level from `info` to `debug`.") TrainRuns.set_log_level(Settings(verbosity = :debug))
    @test_logs (:debug, "debug") (:info,  "info") (:warn,  "warn") (:error, "error") min_level=Logging.Debug log_messages()
    #
    @test_logs (:info, "Changing log level from `debug` to `warn`.") TrainRuns.set_log_level(Settings(verbosity = :warn))
    @test_logs (:warn,  "warn") (:error, "error") min_level=Logging.Warn log_messages()
    #
    @test_logs (:info, "Changing log level from `warn` to `error`.") TrainRuns.set_log_level(Settings(verbosity = :error))
    @test_logs (:error, "error") min_level=Logging.Error log_messages()
    #
    @test_logs (:info, "Changing log level from `error` to `info`.") TrainRuns.set_log_level(Settings(verbosity = :info))
    @test_logs (:info,  "info") (:warn,  "warn") (:error, "error") min_level=Logging.Info log_messages()
end
