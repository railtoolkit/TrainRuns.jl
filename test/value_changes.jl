@testset "value_changes" begin
    @test TrainRuns.value_changes([10, 20, 30]) == [true, true, true]
    @test TrainRuns.value_changes([10, 10, 10]) == [true, false, false]
    @test TrainRuns.value_changes([10, 10, 20]) == [true, false, true]
    @test TrainRuns.value_changes([10]) == [true]
    @test TrainRuns.value_changes([]) == []
end
