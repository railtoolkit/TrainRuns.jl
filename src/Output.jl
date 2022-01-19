module Output

using CSV, DataFrames, Dates

export createOutputDict

function createOutputDict(train::Dict, settings::Dict, path::Dict, movingSection::Dict, drivingCourse::Vector{Dict})
    # method of function createOutputDict for one operation mode
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

function createOutputDict(train::Dict, settings::Dict, path::Dict, MS_MinTime::Dict, DC_MinTime::Vector{Dict}, MS_MinEnergy::Dict, DC_MinEnergy::Vector{Dict})
    # method of function createOutputDict for two operation modes
    outputDict=Dict{Symbol,Any}()
    merge!(outputDict, Dict(:train => train, :path => path, :settings => settings))

    # adding moving sections and drving courses for different operation modes
    if settings[:operationModeMinimumRunningTime] == true
        merge!(outputDict, Dict(:movingSectionMinimumRunningTime => MS_MinTime, :drivingCourseMinimumRunningTime => DC_MinTime))
    end
    if settings[:operationModeMinimumEnergyConsumption] == true
        merge!(outputDict, Dict(:movingSectionMinimumEnergyConsumption => MS_MinEnergy, :drivingCourseMinimumEnergyConsumption => DC_MinEnergy))
    end

    return outputDict
end # function createOutputDict

end # module Output
