using TrainRuns, Test, DataFrames

@testset verbose=true "utils" begin
    @testset "df_2_vector" begin
        @test TrainRuns.df_2_vector(DataFrame()) == Dict[]
        @test TrainRuns.df_2_vector(DataFrame(a = [12], b = ["test"])) ==
              [Dict(:a => 12, :b => "test")]
        @test TrainRuns.df_2_vector(DataFrame(a = [12, 34], b = ["test1", "test2"])) ==
              [Dict(:a => 12, :b => "test1"), Dict(:a => 34, :b => "test2")]
    end

    @testset "value_changes" begin
        @test TrainRuns.value_changes([10, 20, 30]) == [true, true, true]
        @test TrainRuns.value_changes([10, 10, 10]) == [true, false, false]
        @test TrainRuns.value_changes([10, 10, 20]) == [true, false, true]
        @test TrainRuns.value_changes([10]) == [true]
        @test TrainRuns.value_changes([]) == []
    end
end
