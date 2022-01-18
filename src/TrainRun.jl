module TrainRun

# include main module TrainRunCalc
include("./TrainRunCalc.jl")
#include("./types.jl")
#include("./Input.jl")
#include("./Preparation.jl")
#include("./OperationModes.jl")
#include("./Output.jl")

# include additional modules
include("./Import.jl")

# use main module TrainRunCalc
using .TrainRunCalc
#using .types
#using .Input
#using .Preparation
#using .OperationModes
#using .Output

# use additional modules
using .Import

# export main function
export calculateDrivingDynamics,

# export the import functions
importYamlFiles, importYamlFile

# approximationLevel = 6  # value for approximation to intersections
    # TODO:  define it here and give it to each function? (MovingPhases, EnergySaving, ..)

end # module TrainRun
