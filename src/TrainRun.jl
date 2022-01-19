module TrainRun

# include main module TrainRunCalc
include("./TrainRunCalc.jl")

# include additional modules
include("./Import.jl")
include("./Export.jl")

# include additional modules that are not recommended to use in this state
include("./AdditionalOutput.jl")

# use main module TrainRunCalc
using .TrainRunCalc

# use additional modules
using .Import
using .Export

# use additional modules that are not recommended to use in this state
using .AdditionalOutput

# export main function
export calculateDrivingDynamics,

# export the import functions
importYamlFiles, importYamlFile,

# export the export functions
exportToCsv,

# export functions for visualising results that are not recommended to use in this state
plotResults, plotDrivingCourse, printImportantValues, printSectionInformation

# approximationLevel = 6  # value for approximation to intersections
    # TODO:  define it here and give it to each function? (MovingPhases, EnergySaving, ..)

end # module TrainRun
