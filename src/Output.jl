module Output

export createOutputDict

function createOutputDict(train::Dict, settings::Dict, path::Dict, movingSection::Dict, drivingCourse::Vector{Dict})
    outputDict = Dict{Symbol,Any}()
    merge!(outputDict, Dict(:train => train, :path => path, :settings => settings))


    # add moving section and driving courses
    if settings[:operationModeMinimumRunningTime] == true
        merge!(outputDict, Dict(:movingSectionMinimumRunningTime => movingSection,
                                :drivingCourseMinimumRunningTime => drivingCourse))
    elseif settings[:operationModeMinimumEnergyConsumption] == true
        merge!(outputDict, Dict(:movingSectionMinimumEnergyConsumption => movingSection,
                                :drivingCourseMinimumEnergyConsumption => drivingCourse))
    end

    # add points of interest
    if haskey(path, :pointsOfInterest)
        pointsOfInterest = Vector{Dict}()
        POI = 1
        i = 1
        while(POI <= length(path[:pointsOfInterest]) && i <= drivingCourse[end][:i])
            if path[:pointsOfInterest][POI] == drivingCourse[i][:s]
                push!(pointsOfInterest, drivingCourse[i])
                POI = POI+1
            end
            i = i+1
        end

        if settings[:operationModeMinimumRunningTime] == true
            merge!(outputDict, Dict(:pointsOfInterestMinimumRunningTime => pointsOfInterest))
        elseif settings[:operationModeMinimumEnergyConsumption] == true
            merge!(outputDict, Dict(:pointsOfInterestMinimumEnergyConsumption => pointsOfInterest))
        end
    end

    return outputDict
end # function createOutputDict

end # module Output
