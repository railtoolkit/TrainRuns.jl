using DataFrames

@testset "df_2_vector" begin
    @test TrainRuns.df_2_vector(DataFrame()) == Dict[]
    @test TrainRuns.df_2_vector(DataFrame(a = [12], b = ["test"])) ==
          [Dict(:a => 12, :b => "test")]
    @test TrainRuns.df_2_vector(DataFrame(a = [12, 34], b = ["test1", "test2"])) ==
          [Dict(:a => 12, :b => "test1"), Dict(:a => 34, :b => "test2")]
end
