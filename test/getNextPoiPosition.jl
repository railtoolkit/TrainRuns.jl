#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.10.0
# __author__        = "Gregor Wehrle"
# __copyright__     = "2024"
# __license__       = "ISC"

@testset "getNextPoiPosition" begin
    positions::Vector{Real} = [10, 20, 30]
    @test TrainRuns.getNextPoiPosition(positions, 15.0) == 20.0
    @test TrainRuns.getNextPoiPosition(positions, 5.0) == 10.0

    @test_throws "ERROR in getNextPointOfInterest: There is no POI higher than s=35 m." TrainRuns.getNextPoiPosition(
        positions, 35)
end