module Output

export createOutputDict

function createOutputDict(train::Dict, settings::Dict, path::Dict, movingSection::Dict, drivingCourse::Vector{Dict})
    outputDict=Dict{Symbol,Any}()
    merge!(outputDict, Dict(:train => train, :path => path, :settings => settings))

    # adding moving section and drving courses
    if settings[:operationModeMinimumRunningTime] == true
        merge!(outputDict, Dict(:movingSectionMinimumRunningTime => movingSection, :drivingCourseMinimumRunningTime => drivingCourse))
    elseif settings[:operationModeMinimumEnergyConsumption] == true
        merge!(outputDict, Dict(:movingSectionMinimumEnergyConsumption => movingSection, :drivingCourseMinimumEnergyConsumption => drivingCourse))
    end

    return outputDict
end # function createOutputDict

end # module Output
