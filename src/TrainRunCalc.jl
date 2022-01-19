module TrainRunCalc

# include modules of TrainRunCalc
include("./types.jl")
include("./Input.jl")
include("./Preparation.jl")
include("./OperationModes.jl")
include("./Output.jl")


# use modules of TrainRunCalc
using .types
using .Input
using .Preparation
using .OperationModes
using .Output

# export main function
export calculateDrivingDynamics

# approximationLevel = 6  # value for approximation to intersections
    # TODO:  define it here and give it to each function? (MovingPhases, EnergySaving)

# Calculate the driving dynamics of a train run on a path with special settings with information from the corresponding YAML files with the file paths `trainDirectory`, `pathDirectory`, `settingsDirectory`.

"""
    calculateDrivingDynamics(train::Dict, path::Dict, settings::Dict)

Calculate the driving dynamics of a train run on a path with special settings with information from the corresponding dictionaries `train`, `path`, `settings`.

# Examples
```julia-repl
julia> calculateDrivingDynamics(trainDict, pathDict, settingsDict)
todo !!!
```
"""
function calculateDrivingDynamics(train::Dict, path::Dict, settings::Dict)
    # check the input data
    (train, path, settings) = checkAndSetInput!(train, path, settings)
    println("The input has been checked.")

    # prepare the input data
    movingSection=preparateSections(path, train, settings)
    println("The moving section has been prepared.")

    # calculate the train run for oparation mode "minimum running time"
    if settings[:operationModeMinimumRunningTime] ==true || settings[:operationModeMinimumEnergyConsumption] ==true
        (movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime)=calculateMinimumRunningTime!(movingSection, settings, train)
        println("The driving course for the shortest running time has been calculated.")
    end #if


    # calculate the train run for oparation mode "minimum energy consumption"
    if settings[:operationModeMinimumEnergyConsumption] == true
        (movingSectionMinimumEnergyConsumption, drivingCourseMinimumEnergyConsumption)=calculateMinimumEnergyConsumption(movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime, settings, train)
        println("The driving course for the lowest energy consumption has been calculated.")

    # summarize data and create an output dictionary
        output = createOutputDict(train, settings, path, movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime, movingSectionMinimumEnergyConsumption, drivingCourseMinimumEnergyConsumption)
    elseif settings[:operationModeMinimumRunningTime] ==true
        output = createOutputDict(train, settings, path, movingSectionMinimumRunningTime, drivingCourseMinimumRunningTime)
    else
        output = Dict()
    end #if
    # println("The output dictionary has been created.")

    return output
end # function calculateDrivingDynamics

end # module TrainRunCalc
