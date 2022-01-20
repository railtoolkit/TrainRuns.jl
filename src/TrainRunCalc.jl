module TrainRunCalc

# include modules of TrainRunCalc
include("./Input.jl")
include("./Preparation.jl")
include("./Behavior.jl")
include("./Output.jl")


# use modules of TrainRunCalc
using .Input
using .Preparation
using .Behavior
using .Output

# export main function
export calculateDrivingDynamics

approximationLevel = 6  # value for approximation to intersections and precisely calculated digits
    # TODO:  define it here and give it to each function? (Behavior, ...)

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
    movingSection = preparateSections(path, train, settings)
    println("The moving section has been prepared.")

    # calculate the train run for oparation mode "minimum running time"
    if settings[:operationModeMinimumRunningTime] || settings[:operationModeMinimumEnergyConsumption]
        (movingSection, drivingCourse)=calculateMinimumRunningTime!(movingSection, settings, train)
        println("The driving course for the shortest running time has been calculated.")

        # summarize data and create an output dictionary
        output = createOutputDict(train, settings, path, movingSection, drivingCourse)
    else
        output = Dict()
    end #if

    return output
end # function calculateDrivingDynamics

# calculate a train run focussing on using the minimum possible running time
function calculateMinimumRunningTime!(movingSection::Dict, settings::Dict, train::Dict)
   CSs::Vector{Dict} = movingSection[:characteristicSections]

   startingPoint=createDataPoint()
   startingPoint[:i]=1
   startingPoint[:s]=CSs[1][:s_entry]
   drivingCourse::Vector{Dict} = [startingPoint]    # List of data points

   #    for CS in CSs
   for csId in 1:length(CSs)
       # check if the CS has a cruising section
       CS = CSs[csId]
       BSs = CS[:behaviorSections]

       s_breakFree = get(BSs, :breakFree, Dict(:length=>0.0))[:length]
       s_clearing = get(BSs, :clearing, Dict(:length=>0.0))[:length]
       s_acceleration = get(BSs, :acceleration, Dict(:length=>0.0))[:length]
       s_braking = max(0.0, ceil((CS[:v_exit]^2-CS[:v_peak]^2)/2/train[:a_braking], digits=approximationLevel))   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors

       # calculate the cruising sections length
       s_cruising = CS[:length] - s_breakFree - s_clearing - s_acceleration - s_braking

       # reset the characteristic section (CS), delete behavior sections (BS) that were used during the preperation for setting v_entry, v_peak and v_exit
           # 01/07 old: delete!(BSs, :breakFree)
           # 01/07 old: delete!(BSs, :clearing)
           # 01/07 old: delete!(BSs, :acceleration)
           # 01/07 old: delete!(BSs, :diminishing)
           # 01/07 old: delete!(BSs, :cruising)
       CS[:behaviorSections] = Dict()
       CS[:E] = 0.0
       CS[:t] = 0.0


       if s_clearing == CS[:length]
               # 09/06 TODO: thought about using "cruising" because it is used in EnergySaving and not clearing (CS, drivingCourse)=addCruisingSection!(CS, drivingCourse, s_clearing, settings, train, CSs, "cruising")
           (CS, drivingCourse)=addCruisingSection!(CS, drivingCourse, s_clearing, settings, train, CSs, "clearing")
       elseif s_cruising == CS[:length]
           (CS, drivingCourse)=addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "cruising")
       elseif s_cruising > 0.0 || s_braking == 0.0
       # 09/21 elseif s_cruising > 0.0
       # 09/21 elseif s_cruising > 0.01 # if the cruising section is longer than 1 cm (because of rounding issues not >0.0)
           if drivingCourse[end][:v] < CS[:v_peak]
               (CS, drivingCourse)=addAccelerationSection!(CS, drivingCourse, settings, train, CSs)
           end #if

           if CS[:s_exit]-drivingCourse[end][:s]-max(0.0, (CS[:v_exit]^2-drivingCourse[end][:v]^2)/2/train[:a_braking]) < -0.001   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors
               println("ERROR: After accelerating in CS ",csId," the braking distance is too short!")
               println("     before acceleration in CS",csId, "  with s=",drivingCourse[end][:s],"  s_braking=",((CS[:v_exit]^2-drivingCourse[end][:v]^2)/2/train[:a_braking]),"   s_exit=",CS[:s_exit])
               println("                             and v=",drivingCourse[end][:v],"   v_peak=",CS[:v_peak],"  v_exit=",CS[:v_exit])
           end

           s_braking=max(0.0, ceil((CS[:v_exit]^2-drivingCourse[end][:v]^2)/2/train[:a_braking], digits=approximationLevel))   # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors
           s_cruising=CS[:s_exit]-drivingCourse[end][:s]-s_braking

           if s_cruising > 0.0
               (CS, drivingCourse)=addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "cruising")
           end
       else

           if CS[:v_entry] < CS[:v_peak] || s_acceleration > 0.0 # or instead of " || s_acceleration > 0.0" use "v_entry <= v_peak" or "v_i <= v_peak"
           # 09/09 old (not sufficient for steep gradients): if CS[:v_entry] < CS[:v_peak]
               (CS, drivingCourse)=addAccelerationSectionUntilBraking!(CS, drivingCourse, settings, train, CSs)
           end #if
       end #if


       s_braking=max(0.0, ceil((CS[:v_exit]^2-drivingCourse[end][:v]^2)/2/train[:a_braking], digits=approximationLevel))     # ceil is used to be sure that the train reaches v_exit at s_exit in spite of rounding errors

       if drivingCourse[end][:v] > CS[:v_exit]
           #(CS, drivingCourse)=addBrakingSection!(CS, drivingCourse, settings[:massModel], train, CSs)
           (CS, drivingCourse)=addBrakingSection!(CS, drivingCourse, settings, train, CSs)
       end #if

       #= 09/20 old and should never be used:
       if drivingCourse[end][:s] < CS[:s_exit]
           if haskey(BSs, :cruising)
               println("INFO: A second cruising section has been added to CS ", csId," from s=",drivingCourse[end][:s],"  to s_exit=",CS[:s_exit])
           end
           (CS, drivingCourse)=addCruisingSection!(CS, drivingCourse, s_cruising, settings, train, CSs, "cruising")
       end =#
   end #for

   (CSs[end], drivingCourse)=addStandstill!(CSs[end], drivingCourse, settings, train, CSs)

   movingSection[:t] = drivingCourse[end][:t]            # total running time (in s)
   movingSection[:E] = drivingCourse[end][:E]            # total energy consumption (in Ws)

   return (movingSection, drivingCourse)
end #function calculateMinimumRunningTime

end # module TrainRunCalc
