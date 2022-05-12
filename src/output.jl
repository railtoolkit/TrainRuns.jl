#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

function createOutput(train::Train, settings::Settings, path::Path, movingSection::Dict, drivingCourse::Vector{Dict})
    if settings.outputDetail == :running_time
        output = movingSection[:t]  # TODO: or use drivingCourse[end][:t]

    elseif settings.outputDetail == :points_of_interest
        # add points of interest
        if !isempty(path.poi)

            # for elem in 1:length(driving_course)
            # end

            output = Dict[]
            POI = 1
            i = 1
            while POI <= length(path.poi) && i <= drivingCourse[end][:i]
                if path.poi[POI][:station] == drivingCourse[i][:s]
                    push!(output, drivingCourse[i])
                    POI = POI+1
                end
                i = i+1
            end
        end

    elseif settings.outputDetail == :driving_course
            output = drivingCourse

    elseif settings.outputDetail == :everything
        output = Dict{Symbol,Any}()
        merge!(output, Dict(:train => train, :path => path, :settings => settings))


        # add moving section and driving courses
        if settings[:operationModeMinimumRunningTime] == true
            merge!(output, Dict(:movingSectionMinimumRunningTime => movingSection,
                                    :drivingCourseMinimumRunningTime => drivingCourse))
        elseif settings[:operationModeMinimumEnergyConsumption] == true
            merge!(output, Dict(:movingSectionMinimumEnergyConsumption => movingSection,
                                    :drivingCourseMinimumEnergyConsumption => drivingCourse))
        end

        # add points of interest
        if !isempty(path.poi)
            pointsOfInterest = Vector{Dict}()
            POI = 1
            i = 1
            while POI <= length(path.poi) && i <= drivingCourse[end][:i]
                if path.poi[POI] == drivingCourse[i][:s]
                    push!(pointsOfInterest, drivingCourse[i])
                    POI = POI+1
                end
                i = i+1
            end

            if settings[:operationModeMinimumRunningTime] == true
                merge!(output, Dict(:pointsOfInterestMinimumRunningTime => pointsOfInterest))
            elseif settings[:operationModeMinimumEnergyConsumption] == true
                merge!(output, Dict(:pointsOfInterestMinimumEnergyConsumption => pointsOfInterest))
            end
        end
    else
        output = nothing
    end
    return output
end

#=
function createOutputDict(train::Train, settings::Settings, path::Path, movingSection::Dict, drivingCourse::Vector{Dict})
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
    if !isempty(path.poi)
        pointsOfInterest = Vector{Dict}()
        POI = 1
        i = 1
        while POI <= length(path.poi) && i <= drivingCourse[end][:i]
            if path.poi[POI] == drivingCourse[i][:s]
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
=#
