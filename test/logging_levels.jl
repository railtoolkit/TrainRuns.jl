#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2024"
# __license__       = "ISC"

using Logging

function log_messages()
    # @logmsg TrainRuns.Trace "trace" # currently no macro for testing self defined loglevels
    @debug "debug"
    @info "info"
    @warn "warn"
    @error "error"
    # @logmsg TrainRuns.Fatal "fatal" # currently no macro for testing self defined loglevels
end

@testset "logging levels" begin
    logger = ConsoleLogger(min_level=TrainRuns.get_loglevel(Settings(verbosity = :trace)))
    # currently no macro for testing self defined loglevels
    # TODO: add @test_logs (:trace, "trace") (:fatal, "fatal")
    with_logger(logger) do
        @test_logs (:debug, "debug") (:info, "info") (:warn, "warn") (:error, "error") min_level=TrainRuns.Trace log_messages()
    end

    @test TrainRuns.get_loglevel(Settings(verbosity = :trace)) == LogLevel(-2000)
    @test TrainRuns.get_loglevel(Settings(verbosity = :debug)) == LogLevel(-1000)
    @test TrainRuns.get_loglevel(Settings(verbosity = :info)) == LogLevel(0)
    @test TrainRuns.get_loglevel(Settings()) == LogLevel(0)
    @test TrainRuns.get_loglevel(Settings(verbosity = :warn)) == LogLevel(1000)
    @test TrainRuns.get_loglevel(Settings(verbosity = :error)) == LogLevel(2000)
    @test TrainRuns.get_loglevel(Settings(verbosity = :fatal)) == LogLevel(3000)
end
