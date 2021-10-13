module RailwayDrivingDynamics

include("./types.jl")
include("./RailwayDrivingDynamicsInput.jl")
include("./RailwayDrivingDynamicsPreparation.jl")
include("./RailwayDrivingDynamicsOperationModes.jl")
include("./RailwayDrivingDynamicsOutput.jl")


using .types
using .RailwayDrivingDynamicsInput
using .RailwayDrivingDynamicsPreparation
using .RailwayDrivingDynamicsOperationModes
using .RailwayDrivingDynamicsOutput

export calculateDrivingDynamics

function calculateDrivingDynamics(vehicleFilePath::String, pathFilePath::String, settingsFilePath::String)
# main function for calculating the driving dynamics
    println("")
    println("")
    println("")

    # input
    (vehicle, path, settings)=readInput(vehicleFilePath, pathFilePath, settingsFilePath)
    println("The input has been saved.")


    # preparing the input data
    movingSection=preparateSections(path, vehicle, settings)
    println("The moving section has been preparated.")

    if settings.operationModeMinimumRunningTime==true || settings.operationModeMinimumEnergyConsumption==true
        (movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime)=simulateMinimumRunningTime!(movingSection, settings, vehicle)
       # println("t_total=", drivingCourseMinimumRunningTime[end].t)
       # printSectionInformation(movingSectionMinimumRunningTime)
        println("The driving course for the shortest running time has been calculated.")
    end #if


    # oparation mode "minimum energy consumption"
    if settings.operationModeMinimumEnergyConsumption==true
        (movingSectionMinimumEnergyConsumption, drivingCourseMinimumEnergyConsumption)=simulateMinimumEnergyConsumption(movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime, settings, vehicle)
       # printSectionInformation(movingSectionMinimumEnergyConsumption)
        println("The driving course for the energy consumption has been calculated.")
    end #if

    #output
    if settings.operationModeMinimumRunningTime==true && settings.operationModeMinimumEnergyConsumption==true
        plotDrivingCourse(drivingCourseMinimumRunningTime, drivingCourseMinimumEnergyConsumption)
        return createOutput(settings, path.name, vehicle.name, drivingCourseMinimumRunningTime, movingSectionMinimumRunningTime, drivingCourseMinimumEnergyConsumption, movingSectionMinimumEnergyConsumption)
    elseif settings.operationModeMinimumRunningTime==true
        plotDrivingCourse(drivingCourseMinimumRunningTime)
        return createOutput(settings, path.name, vehicle.name, drivingCourseMinimumRunningTime, movingSectionMinimumRunningTime)
    elseif settings.operationModeMinimumEnergyConsumption==true
        plotDrivingCourse(drivingCourseMinimumEnergyConsumption)
        return createOutput(settings, path.name, vehicle.name, drivingCourseMinimumEnergyConsumption, movingSectionMinimumEnergyConsumption)
    else
        println("No Output was demanded. So no output is created.")
        return Dict()
    end
end # function calculateDrivingDynamics

end # module RailwayDrivingDynamics
