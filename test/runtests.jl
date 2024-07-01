#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Martin Scheidt"
# __copyright__     = "2024"
# __license__       = "ISC"

using TrainRuns, Test, CSV

"""
    @df_snapshot_test(snapshot_path, expr::Expr)

Saves DataFrames to CSV and compares them.
To renew snapshot delete the file named like the test.

# Example
```julia-repl
julia> @df_snapshot_test "category/test_name" test_df()
```
This test will check the output of 'test_df()' against a known 'test_name.csv' in the test folder of 'snapshot/category'.
"""
macro df_snapshot_test(snapshot_path, expr::Expr)
    return quote
        filename = $(esc(snapshot_path)) * ".csv"
        filepath = joinpath(Base.source_dir(), "snapshots", filename)
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

@testset verbose=true "TrainRuns" begin
    include("logging_levels.jl")
    include("df_2_vector.jl")
    include("getNextPoiPosition.jl")
    include("getOutputByDetail.jl")
    include("highlevel_test.jl")
    include("value_changes.jl")
end
