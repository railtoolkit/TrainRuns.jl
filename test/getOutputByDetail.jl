using DataFrames

@testset "getOutputByDetail" begin
    drivingCourse = DataFrame([
        (s = 0, t = 0, behavior = "breakFree"),
        (s = 10, t = 10, behavior = "accelerating"),
        (s = 20, t = 20, behavior = "accelerating"),
        (s = 30, t = 30, behavior = "halt")
    ])

    pois = DataFrame([
        (s = 10, label = "signal", nonstandard = 123)
    ])

    pois_doublepos = DataFrame([
        (s = 10, label = "signal1", nonstandard = 123),
        (s = 10, label = "signal2", nonstandard = 456)
    ])

    @testset "running_time" begin
        @test TrainRuns.getOutputByDetail(drivingCourse, pois, :running_time) ==
              DataFrame(t = [30])
    end

    @testset "points_of_interest" begin
        @test TrainRuns.getOutputByDetail(drivingCourse, pois, :points_of_interest) ==
              DataFrame(s = [10], t = [10], behavior = ["accelerating"],
            label = ["signal"], nonstandard = [123])
        @test TrainRuns.getOutputByDetail(
            drivingCourse, pois_doublepos, :points_of_interest) ==
              DataFrame(
            s = [10, 10], t = [10, 10], behavior = ["accelerating", "accelerating"],
            label = ["signal1", "signal2"], nonstandard = [123, 456])
        @test TrainRuns.getOutputByDetail(
            drivingCourse, DataFrame(), :points_of_interest) ==
              DataFrame(s = [0, 30], t = [0, 30],
            behavior = ["breakFree", "halt"], label = ["", ""])
    end

    @testset "data_points" begin
        @test TrainRuns.getOutputByDetail(drivingCourse, pois, :data_points) ==
              DataFrame(s = [0, 10, 30], t = [0, 10, 30],
            behavior = ["breakFree", "accelerating", "halt"],
            label = ["", "signal", ""])
    end

    @testset "driving_course" begin
        @test TrainRuns.getOutputByDetail(drivingCourse, pois, :driving_course) ==
              DataFrame(s = [0, 10, 20, 30], t = [0, 10, 20, 30],
            behavior = ["breakFree", "accelerating", "accelerating", "halt"],
            label = ["", "signal", "", ""])
    end
end
