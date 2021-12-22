module Input

import YAML
using ..types

export readInput

"""
Read the input information from YAML files for train, path and settings, save it in different dictionaries and return them.
"""
function readInput(trainDirectory::String, pathDirectory::String, settingsDirectory::String)
     train=inputTrain(trainDirectory)
     path=inputPath(pathDirectory)
     settings=inputSettings(settingsDirectory)

     return (train, path, settings)
 end #function readInput

"""
Read the train information from a YAML file, save it in a train Dict and return it.
"""
function inputTrain(trainDirectory::String)
    data = YAML.load(open(trainDirectory))
    #collect(keys(data))
    #collect(values(data))

    if haskey(data["train"],"name")
        name=data["train"]["name"]                            # train's name
        delete!(data["train"], "name")
    else
        error("ERROR at reading the train yaml file: The keyword name is missing. It has to be added.")
    end

    id=1                                # trains identifier

    if haskey(data["train"],"trainType")
            if typeof(data["train"]["trainType"])==String && (data["train"]["trainType"]=="freight" || data["train"]["trainType"]=="motor coach train" || data["train"]["trainType"]=="passenger")
            trainType=data["train"]["trainType"]                  # "passenger" or "freight" or "motor coach train"
            delete!(data["train"], "trainType")
        else
            error("ERROR at reading the train yaml file: The value of trainType is wrong. It has to be freight, motor coach train or passenger.")
        end
    else
        error("ERROR at reading the train yaml file: The keyword trainType is missing. It has to be added with the value freight, motor coach train or passenger.")
    end

    if haskey(data["train"],"l_train")
        if typeof(data["train"]["l_train"]) <: Real && data["train"]["l_train"]>0.0
            trainLength=data["train"]["l_train"]                                                # total length (in m)
            delete!(data["train"], "l_train")
        else
            error("ERROR at reading the train yaml file: The value of the length is no real number >0.0.")
        end
    else
        error("ERROR at reading the train yaml file: The keyword length is missing. It has to be added with a value of type real floating point number >0.0.")
    end


    # speed limit:                                                                      # trains speed limit (in m/s)
    v_limit_temp=0.0
    v_limit_kmh_temp=0.0
    if haskey(data["train"],"v_limit") && data["train"]["v_limit"]!=nothing
        if typeof(data["train"]["v_limit"]) <: Real && data["train"]["v_limit"]>0.0
            v_limit_temp=data["train"]["v_limit"]                # trains speed limit (in m/s)
            delete!(data["train"], "v_limit")
        else
            error("ERROR at reading the train yaml file: The value of v_limit is no real floating point number >0.0.")
        end
    end

    if haskey(data["train"],"v_limit_kmh") && data["train"]["v_limit_kmh"]!=nothing
        if typeof(data["train"]["v_limit_kmh"]) <: Real && data["train"]["v_limit_kmh"]>0.0
            v_limit_kmh_temp=data["train"]["v_limit_kmh"]                # trains speed limit (in km/h)
            delete!(data["train"], "v_limit_kmh")
        else
            error("ERROR at reading the train yaml file: The value of v_limit is no real floating point number >0.0.")
        end
    end

    if v_limit_temp > 0.0 && v_limit_kmh_temp > 0.0
        v_limit=v_limit_temp
        difference=abs(v_limit_temp-v_limit_kmh_temp/3.6)
        if difference >0.0
            println("WARNING at reading the train yaml file: The values of v_limit and v_limit_kmh differ by ",difference," m/s. The value v_limit=",v_limit," m/s is used." )
        end
    elseif v_limit_temp > 0.0
        v_limit=v_limit_temp
    elseif v_limit_kmh_temp > 0.0
        v_limit=v_limit_kmh_temp/3.6
    else
        v_limit=1000/3.6
        println("WARNING at reading the train yaml file: There is no value for the trains speed limit (v_limit or v_limit_kmh). The value v_limit=1000 km/h =",v_limit," m/s is used." )
    end


    # a_braking
    if haskey(data["train"],"a_braking")
        if typeof(data["train"]["a_braking"]) <: Real
            a_braking=data["train"]["a_braking"]
        else
            error("ERROR at reading the train yaml file: The value of the a_braking is no real floating point number <0.0.")
        end
        delete!(data["train"], "a_braking")

        if a_braking > 0.0
            a_braking =-a_braking
            println("WARNING at reading the train yaml file: The value for a_braking is >0.0. The braking acceleration has to be <0.0. Therefore a_braking =",a_braking," m/s^2 is used." )
        elseif a_braking == 0.0
            error("ERROR at reading the train yaml file: The value for a_braking is 0.0. The braking acceleration has to be <0.0.")
        end
    else
        error("ERROR at reading the train yaml file: The keyword a_braking is missing. It has to be added with a value of type real floating point number <0.0.")
    end

    # mass on the traction units driving axles (in kg)
    if haskey(data["train"],"m_td")
        if typeof(data["train"]["m_td"]) <: Real && data["train"]["m_td"]>0.0
            m_td=data["train"]["m_td"]
        else
            error("ERROR at reading the train yaml file: The value of m_td is no real floating point number >0.0.")
        end
    else
        error("ERROR at reading the train yaml file: The keyword m_td is missing. It has to be added with a value of type real floating point number >0.0.")
    end
    delete!(data["train"], "m_td")


    # mass on the traction units carrying axles (in kg)
    if haskey(data["train"],"m_tc")
        if typeof(data["train"]["m_tc"]) <: Real && data["train"]["m_tc"]>=0.0
            m_tc=data["train"]["m_tc"]
        else
            error("ERROR at reading the train yaml file: The value of m_tc is no real floating point number >=0.0.")
        end
    else
        error("ERROR at reading the train yaml file: The keyword m_tc is missing. It has to be added with a value of type real floating point number >=0.0.")
    end
    delete!(data["train"], "m_tc")

    # mass of the traction unit (in kg)
    m_t=m_td+m_tc


    # mass of the set of wagons (consist)  (in kg)
    if haskey(data["train"],"m_w")
        if typeof(data["train"]["m_w"]) <: Real && data["train"]["m_w"]>=0.0
            m_w=data["train"]["m_w"]
        else
            error("ERROR at reading the train yaml file: The value of m_w is no real floating point number >=0.0.")
        end
    else
        m_w=0.0
        println("WARNING at reading the train yaml file: The keyword m_w is missing. Therefore m_w =",m_w," kg is used.")
    end
    delete!(data["train"], "m_w")

    # total mass (in kg)
    m_train=m_t+m_w

    if haskey(data["train"],"rotationMassFactor_train") && typeof(data["train"]["rotationMassFactor_train"]) <: Real
        if data["train"]["rotationMassFactor_train"]>0.0
            ξ_train=data["train"]["rotationMassFactor_train"]
            ξ_t=0.0
            ξ_w=0.0
        else
            error("ERROR at reading the train yaml file: The value of rotationMassFactor_train is no real floating point number >0.0.")
        end
    elseif haskey(data["train"],"rotationMassFactor_t") && typeof(data["train"]["rotationMassFactor_t"]) <: Real && (m_w==0.0 || (haskey(data["train"],"rotationMassFactor_w") && typeof(data["train"]["rotationMassFactor_w"]) <: Real))
        if data["train"]["rotationMassFactor_t"]>0.0
            ξ_t=data["train"]["rotationMassFactor_t"]
        else
            error("ERROR at reading the train yaml file: The value of rotationMassFactor_t is no real floating point number >0.0.")
        end
        if m_w>0.0
            if data["train"]["rotationMassFactor_w"]>=0.0
                ξ_w=data["train"]["rotationMassFactor_w"]
            else
                error("ERROR at reading the train yaml file: The value of rotationMassFactor_w is no real floating point number >=0.0.")
            end
        else
            ξ_w=0.0
        end
        ξ_train=(ξ_t*m_t + ξ_w*m_w)/m_train  # rotation mass factor of the whole train (without unit)
    else
        error("ERROR at reading the train yaml file: The keywords rotationMassFactor_train or rotationMassFactor_t and rotationMassFactor_w are missing. They has to be added with a value of type real floating point number.")
    end
    delete!(data["train"], "rotationMassFactor_train")
    delete!(data["train"], "rotationMassFactor_t")
    delete!(data["train"], "rotationMassFactor_w")


    # pairs of velocity and tractive effort
    if haskey(data["train"],"F_T_pairs") && data["train"]["F_T_pairs"]!=nothing
        F_T_pairs=data["train"]["F_T_pairs"]

        tractiveEffortVelocityPairs=checkAndDefineTractiveEffortInput(F_T_pairs, 1.0)

        if haskey(data["train"],"F_T_pairs_kmh") && data["train"]["F_T_pairs_kmh"]!=nothing
            println("WARNING at reading the train yaml file: There are values for F_T_pairs and F_T_pairs_kmh. The values for F_T_pairs are used." )
        end

    elseif haskey(data["train"],"F_T_pairs_kmh") && data["train"]["F_T_pairs_kmh"]!=nothing
        F_T_pairs_kmh=data["train"]["F_T_pairs_kmh"]

        tractiveEffortVelocityPairs=checkAndDefineTractiveEffortInput(F_T_pairs_kmh, 1000/3600)
    else
        error("ERROR at reading the train yaml file: There has to be one of the keywords F_T_pairs or F_T_pairs_kmh filled with a list of pairs of velocity and tractive effort.")
    end # if
    delete!(data["train"], "F_T_pairs")
    delete!(data["train"], "F_T_pairs_kmh")


    # coefficients for the vehicle resistance of the traction unit

    # coefficient for velocitiy difference between traction unit and outdoor air
    Δv_t=15.0/3.6

    # coefficient for basic resistance due to the traction units driving axles (in ‰)
    if haskey(data["train"],"f_Rtd0") && data["train"]["f_Rtd0"]!=nothing
        if typeof(data["train"]["f_Rtd0"]) <: Real && data["train"]["f_Rtd0"]>=0.0
            f_Rtd0=data["train"]["f_Rtd0"]
        else
            error("ERROR at reading the train yaml file: The value of f_Rtd0 is no real floating point number >=0.0.")
        end
    else
        f_Rtd0=0.0
        println("WARNING at reading the train yaml file: The keyword f_Rtd0 is missing. Therefore f_Rtd0=0.0 ‰ will be assumed and used." )
    end
    delete!(data["train"], "f_Rtd0")

    # coefficient for basic resistance due to the traction units carring axles (in ‰)
    if haskey(data["train"],"f_Rtc0") && data["train"]["f_Rtc0"]!=nothing
        if typeof(data["train"]["f_Rtc0"]) <: Real && data["train"]["f_Rtc0"]>=0.0
            f_Rtc0=data["train"]["f_Rtc0"]
        else
            error("ERROR at reading the train yaml file: The value of f_Rtc0 is no real floating point number >=0.0.")
        end
    else
        f_Rtc0=0.0
        println("WARNING at reading the train yaml file: The keyword f_Rtc0 is missing. Therefore f_Rtc0=0.0 ‰ will be assumed and used." )
    end
    delete!(data["train"], "f_Rtc0")

    # coefficient for air resistance of the traction units (in N)
    if haskey(data["train"],"F_Rt2") && data["train"]["F_Rt2"]!=nothing
        if typeof(data["train"]["F_Rt2"]) <: Real && data["train"]["F_Rt2"]>=0.0
            F_Rt2=data["train"]["F_Rt2"]
        else
            error("ERROR at reading the train yaml file: The value of F_Rt2 is no real floating point number >=0.0.")
        end
    else
        F_Rt2=0.0
        println("WARNING at reading the train yaml file: The keyword F_Rt2 is missing. Therefore F_Rt2=0.0 N will be assumed and used." )
    end
    delete!(data["train"], "F_Rt2")



    # coefficients for the vehicle resistance of the set of wagons (consist)

    # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)
    if trainType=="passenger" || trainType=="motor coach train"
        Δv_w=15.0/3.6
    elseif trainType== "freight"
        Δv_w=0.0
    end # if

    # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    if haskey(data["train"],"f_Rw0") && data["train"]["f_Rw0"]!=nothing
        if typeof(data["train"]["f_Rw0"]) <: Real && data["train"]["f_Rw0"]>=0.0
            f_Rw0=data["train"]["f_Rw0"]
        else
            error("ERROR at reading the train yaml file: The value of f_Rw0 is no real floating point number >=0.0.")
        end
    else
        f_Rw0=0.0
        println("WARNING at reading the train yaml file: The keyword f_Rw0 is missing. Therefore f_Rw0=0.0 ‰ will be assumed and used." )
    end
    delete!(data["train"], "f_Rw0")

    # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    if haskey(data["train"],"f_Rw1") && data["train"]["f_Rw1"]!=nothing
        if typeof(data["train"]["f_Rw1"]) <: Real && data["train"]["f_Rw1"]>=0.0
            f_Rw1=data["train"]["f_Rw1"]
        else
            error("ERROR at reading the train yaml file: The value of f_Rw1 is no real floating point number >=0.0.")
        end
    else
        f_Rw1=0.0
        println("WARNING at reading the train yaml file: The keyword f_Rw1 is missing. Therefore f_Rw1=0.0 ‰ will be assumed and used." )
    end
    delete!(data["train"], "f_Rw1")

    # coefficient for basic resistance of the set of wagons (consist) (in ‰)
    if haskey(data["train"],"f_Rw2") && data["train"]["f_Rw2"]!=nothing
        if typeof(data["train"]["f_Rw2"]) <: Real && data["train"]["f_Rw2"]>=0.0
            f_Rw2=data["train"]["f_Rw2"]
        else
            error("ERROR at reading the train yaml file: The value of f_Rw2 is no real floating point number >=0.0.")
        end
    else
        f_Rw2=0.0
        println("WARNING at reading the train yaml file: The keyword f_Rw2 is missing. Therefore f_Rw2=0.0 ‰ will be assumed and used." )
    end
    delete!(data["train"], "f_Rw2")


    # inform the user, which keywords of the input data are not used in this tool:
    if length(data["train"])>0
        println("INFO at reading the train yaml file: The following Keywords are not used in this tool:")
        for key in keys(data["train"])
            println("  - ",key)
        end
    end

    # create the train Dictionary
    train= Dict(:name => name,              # train's name
                :id => id,                  # train's identifier
                :trainType => trainType,    # type of train "passenger" or "freight" or "motor coach train"
                :trainLength => trainLength,        # total length (in m)
                :v_limit => v_limit,        # trains speed limit (in m/s)
                :a_braking => a_braking,    # braking acceleration (in m/s^2)
                :m_train => m_train,        # total mass (in kg)
                :ξ_train => ξ_train,        # rotation mass factor of the whole train (without unit)
                                            # if not available use ξ_t and ξ_w

                # traction unit
                :m_t => m_t,            # mass of the traction unit (in kg)
                :m_td => m_td,          # mass on the traction units driving axles (in kg)
                :m_tc => m_tc,          # mass on the traction units carrying axles (in kg)
                :ξ_t => ξ_t,            # rotation mass factor of the traction unit (without unit)
                                        # in case ξ_train is not available
                :tractiveEffortVelocityPairs => tractiveEffortVelocityPairs,    # list of velocities and their corresponding tractive effort (in [m/s , N])

                :f_Rtd0 => f_Rtd0,      # coefficient for basic resistance due to the traction units driving axles (in ‰)
                :f_Rtc0 => f_Rtc0,      # coefficient for basic resistance due to the traction units carring axles (in ‰)
                :F_Rt2 => F_Rt2,        # coefficient for air resistance of the traction units (in N)
                :Δv_t => Δv_t,          # coefficient for velocitiy difference between traction unit and outdoor air

                # set of wagons
                :m_w => m_w,            # mass of the set of wagons (in kg)
                :ξ_w => ξ_w,            # rotation mass factor of the set of wagons (without unit)
                                        # in case ξ_train is not available
                :f_Rw0 => f_Rw0,        # coefficient for basic resistance of the set of wagons (in ‰)
                :f_Rw1 => f_Rw1,        # coefficient for resistance to rolling of the set of wagons (in ‰)
                :f_Rw2 => f_Rw2,        # coefficient for air resistance of the set of wagons (in ‰)
                :Δv_w => Δv_w)          # coefficient for velocitiy difference between set of wagons and outdoor air (in m/s)

    return train
end #function inputTrain

function checkAndDefineTractiveEffortInput(F_T_pairs, velocityMultiplier::AbstractFloat)
 # TODO: check if its numbers are real ? function checkAndDefineTractiveEffortInput(F_T_pairs::Array{Array{Real,1},1}, velocityMultiplier::AbstractFloat)
    # check if the elements of the array have the correct type
    errorDetected=false
    for row in 1:length(F_T_pairs)
        if typeof(F_T_pairs[row][1]) <: Real && F_T_pairs[row][1]>=0.0
        else
            errorDetected=true
            println("ERROR at reading the train yaml file: The speed value of F_T_pairs in row ", row ," is no real floating point number >=0.0.")
        end
        if typeof(F_T_pairs[row][2]) <: Real && F_T_pairs[row][2]>=0.0
        else
            errorDetected=true
            println("ERROR at reading the train yaml file: The tractive effort value of F_T_pairs in row ", row ," is no real floating point number >=0.0.")
        end

        if row>=2 && F_T_pairs[row][1] <= F_T_pairs[row-1][1]
            errorDetected=true
            println("ERROR at reading the train yaml file: The speed value of F_T_pairs in row ", row ," (v=",F_T_pairs[row][1]," m/s) is not higher than the speed value in the previous row (v=",F_T_pairs[row-1][1]," m/s).")
        end
    end # for
    if errorDetected
        error("ERROR at reading the train yaml file: Only real floating point number >=0.0 are allowed for speed and tractive effort. The speed values have to be listed from low to high.")
    end

    # create tractiveEffortVelocityPairs
    tractiveEffortVelocityPairs=[]
    if F_T_pairs[1][1]>0.0    # if there is no F_T for v=0.0, the first known value is used
        push!(tractiveEffortVelocityPairs, [0.0, F_T_pairs[1][2]])
        println("WARNING at reading the train yaml file: The tractive effort for v=0 m/s is missing. Therefore the first given value F_T(v=",F_T_pairs[1][1]," m/s)=",F_T_pairs[1][2]," N will be used." )
    end

    for row in 1:length(F_T_pairs)
        push!(tractiveEffortVelocityPairs, [F_T_pairs[row][1]*velocityMultiplier, F_T_pairs[row][2]])
    end # for


    if length(F_T_pairs[1])>2
        println("INFO according the train yaml file: Only the first two columns of F_T_pairs are used in this tool.")
    end
    return tractiveEffortVelocityPairs
end #function checkAndDefineTractiveEffortInput


function inputPath(pathDirectory::String)
 # read path information from a YAML file, save it in a path Dict and return it
    data = YAML.load(open(pathDirectory))
    #collect(keys(data))
    #collect(values(data))


    if haskey(data["path"],"name") && data["path"]["name"]!=nothing
        name=data["path"]["name"]                            # path name
    else
        error("ERROR at reading the path yaml file: The keyword name is missing. It has to be added.")
    end
    delete!(data["path"], "name")

    id=1                                                    # path identifier

    # read the section starting positions and corresponding information
    if haskey(data["path"],"sectionStarts") && data["path"]["sectionStarts"]!=nothing
        sectionStartsArray=data["path"]["sectionStarts"]
        conversionFactor=1.0                                # conversion factor between the units m/s and m/s
        valueKey="sectionStarts" 	                   # necessary for error messages

        if haskey(data["train"],"sectionStarts") && data["train"]["sectionStarts_kmh"]!=nothing
            println("WARNING at reading the path yaml file: There are values for sectionStarts and sectionStarts_kmh. The values for sectionStarts are used." )
        end
    elseif haskey(data["path"],"sectionStarts_kmh") && data["path"]["sectionStarts_kmh"]!=nothing
        sectionStartsArray=data["path"]["sectionStarts_kmh"]
        conversionFactor=1/3.6                              # conversion factor between the units km/h and m/s
        valueKey="sectionStarts_kmh" 	                   # necessary for error messages
    else
        error("ERROR at reading the path yaml file: The keyword sectionStarts or sectionStarts_kmh is missing. It has to be added.")
    end # if
    delete!(data["path"], "sectionStarts")
    delete!(data["path"], "sectionStarts_kmh")

    # check if the array is correct and if elements of the array have the correct type and plausible values
    errorDetected=false
    if length(sectionStartsArray)>=2
    else
        error("ERROR at reading the path yaml file: The keyword ",valueKey," needs at least two rows for two points each with the three columns [s, v_limit, gradient].")
    end

    for row in 1:length(sectionStartsArray)
        if length(sectionStartsArray[row])>=3
            if length(sectionStartsArray[row])>3
                println("INFO at reading the path yaml file: Only the first three columns of sectionStartsArray are used in this tool.")
            end
        else
            error("ERROR at reading the path yaml file: The keyword ",valueKey," needs to be filled with the three columns [s, v_limit, gradient].")
        end

        if typeof(sectionStartsArray[row][1]) <: Real
            if row > 1
                if sectionStartsArray[row][1]>sectionStartsArray[row-1][1]
                else
                    errorDetected=true
                    println("ERROR at reading the path yaml file: The postion value of ",valueKey," in row ", row ," (",sectionStartsArray[row][1]," m) has to be higher than the value in the row above (",sectionStartsArray[row-1][1]," m).")
                end
            end
        else
            errorDetected=true
            println("ERROR at reading the path yaml file: The position value (column 1) of ",valueKey," in row ", row ," is no real floating point number.")
        end

        if typeof(sectionStartsArray[row][2]) <: Real && sectionStartsArray[row][2]>=0.0
        else
            errorDetected=true
            println("ERROR at reading the path yaml file: The speed limit (column 2) of ",valueKey," in row ", row ," is no real floating point number >=0.0.")
        end
        if typeof(sectionStartsArray[row][3]) <: Real
        else
            errorDetected=true
            println("ERROR at reading the path yaml file: The tractive effort value (column 3) of ",valueKey," in row ", row ," is no real floating point number.")
        end
    end # for
    if errorDetected
        error("ERROR at reading the path yaml file: The values of ",valueKey," have to be corrected.")
    end

    # save values in the path Dict
    sections=[]
    for row in 2:length(sectionStartsArray)
        s_start=sectionStartsArray[row-1][1]                    # first point of the section (in m)
        s_end=sectionStartsArray[row][1]                        # first point of the next section (in m)
        v_limit=sectionStartsArray[row-1][2]*conversionFactor   # paths speed limt (in m/s)
        f_Rp=sectionStartsArray[row-1][3]                       # specific path resistance of the section (in ‰)
        section=Dict(:s_start => s_start, :s_end => s_end, :v_limit => v_limit, :f_Rp => f_Rp)
        push!(sections, section)
    end # for
    # s_start in first entry defines the path's beginning
    # s_end in last entry defines the path's ending

    path = Dict(:name => name,
                :id => id,
                :sections => sections)

    # inform the user which keywords of the input data are not used in this tool:
    if length(data["path"])>0
        println("INFO at reading the path yaml file: The following Keywords are not used in this tool:")
        for key in keys(data["path"])
            println("  - ",key)
        end
    end
    return path
end # function inputPath



## settings for the calculation
function inputSettings(settingsDirectory::String)
 # read setting information from a YAML file, save it in a settings Dict and return it
    data = YAML.load(open(settingsDirectory))
    #collect(keys(data))
    #collect(values(data))

    # initialize the settings Dictionary
    settings = Dict(:massModel => "",                                   # model type of the unions mass "mass point" or "homogeneous strip"
                    :stepVariable => "",                                # step variable of the step method "s in m", "t in s" or "v in m/s"
                    :stepSize => 0.0,                                   # step size (unit depends on steapVariable s in m, t in s and v in m/s)
                    :operationModeMinimumRunningTime => false,          # operation mode "minimum running time"
                    :operationModeMinimumEnergyConsumption => false,    # operation mode "minimum energy consumption"
                    :typeOfOutput => "",                                # output as "julia dictionary" or as "CSV"
                    :csvDirectory => "",                                # path of the folder in which the CSV files willl be saved
                    :detailOfOutput => "")                              # detail of output "minimal" or "everything"

    # model type of the train's mass "mass point" or "homogeneous strip"
    if haskey(data["settings"],"massModel")
        if typeof(data["settings"]["massModel"])==String && (data["settings"]["massModel"]=="mass point" || data["settings"]["massModel"]=="homogeneous strip")
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:massModel => data["settings"]["massModel"]))    # "mass point" or "homogeneous strip"
            settings[:massModel] = data["settings"]["massModel"]    # "mass point" or "homogeneous strip"
        else
            error("ERROR at reading the settings yaml file: The value of massModel is wrong. It has to be mass point or homogeneous strip.")
        end
    else
        error("ERROR at reading the settings yaml file: The keyword massModel is missing. It has to be added with the value mass point or homogeneous strip.")
    end
    delete!(data["settings"], "massModel")


    # step variable of the step method "s in m", "t in s" or "v in m/s"
    if haskey(data["settings"],"stepVariable")
        if typeof(data["settings"]["stepVariable"])==String && (data["settings"]["stepVariable"]=="s in m" || data["settings"]["stepVariable"]=="t in s" || data["settings"]["stepVariable"]=="v in m/s")
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:stepVariable => data["settings"]["stepVariable"]))      # "s in m", "t in s" or "v in m/s"
            settings[:stepVariable] = data["settings"]["stepVariable"]    # "s in m", "t in s" or "v in m/s"
        else
            error("ERROR at reading the settings yaml file: The value of stepVariable is wrong. It has to be s in m, t in s or v in m/s.")
        end
    else
        error("ERROR for the settings yaml file: The keyword stepVariable is missing. It has to be added with the value s in m, t in s or v in m/s.")
    end
    delete!(data["settings"], "stepVariable")


    # step size (unit depends on steapVariable: s in m, t in s and v in m/s)
    if haskey(data["settings"],"stepSize")
        if typeof(data["settings"]["stepSize"]) <: Real && data["settings"]["stepSize"]>0.0
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:stepSize => data["settings"]["stepSize"]))
            settings[:stepSize] = data["settings"]["stepSize"]
        else
            error("ERROR at reading the settings yaml file: The value of the stepSize is no real floating point number >0.0.")
        end
    else
        error("ERROR at reading the settings yaml file: The keyword stepSize is missing. It has to be added with a value of type real floating point number >0.0.")
    end
    delete!(data["settings"], "stepSize")


    # operation mode "minimum running time"
    if haskey(data["settings"],"operationModeMinimumRunningTime") && data["settings"]["operationModeMinimumRunningTime"]!=nothing
        if typeof(data["settings"]["operationModeMinimumRunningTime"])==Bool
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:operationModeMinimumRunningTime => data["settings"]["operationModeMinimumRunningTime"]))
            settings[:operationModeMinimumRunningTime] = data["settings"]["operationModeMinimumRunningTime"]
        else
            error("ERROR at reading the settings yaml file: The value of the keyword operationModeMinimumRunningTime is not correct. The value has to be of type boolean.")
        end
    else
        settings[:operationModeMinimumRunningTime]=false
        println("WARNING at reading the settings yaml file: The keyword operationModeMinimumRunningTime or its value is missing. Therefore operationModeMinimumRunningTime=",settings[:operationModeMinimumRunningTime]," is assumed and used.")
    end
    delete!(data["settings"], "operationModeMinimumRunningTime")


    # operation mode "minimum energy consumption"
    if haskey(data["settings"],"operationModeMinimumEnergyConsumption") && data["settings"]["operationModeMinimumEnergyConsumption"]!=nothing
        if typeof(data["settings"]["operationModeMinimumEnergyConsumption"])==Bool
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:operationModeMinimumEnergyConsumption => data["settings"]["operationModeMinimumEnergyConsumption"]))
            settings[:operationModeMinimumEnergyConsumption] = data["settings"]["operationModeMinimumEnergyConsumption"]
        else
            error("ERROR at reading the settings yaml file: The value of the keyword operationModeMinimumEnergyConsumption is not correct. The value has to be of type boolean.")
        end
    else
        settings[:operationModeMinimumEnergyConsumption] = false
        println("WARNING at reading the settings yaml file: The keyword operationModeMinimumEnergyConsumption or its value is missing. Therefore operationModeMinimumEnergyConsumption=", settings[:operationModeMinimumEnergyConsumption]," is assumed and used.")
    end
    delete!(data["settings"], "operationModeMinimumEnergyConsumption")

    # output as "julia dictionary" or as "CSV"
    if haskey(data["settings"],"typeOfOutput")
        if typeof(data["settings"]["typeOfOutput"])==String && (data["settings"]["typeOfOutput"]=="julia dictionary" || data["settings"]["typeOfOutput"]=="CSV")
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:typeOfOutput => data["settings"]["typeOfOutput"]))          # "julia dictionary" or "CSV"
            settings[:typeOfOutput] = data["settings"]["typeOfOutput"]    # "julia dictionary" or "CSV"
        else
            error("ERROR at reading the settings yaml file: The value of typeOfOutput is wrong. It has to be julia dictionary or CSV.")
        end
    else
        error("ERROR at reading the settings yaml file: The keyword typeOfOutput is missing. It has to be added with the value julia dictionary or CSV.")
    end
    delete!(data["settings"], "typeOfOutput")

    # TODO: it could be checked if the path is existing on the pc
    if settings[:typeOfOutput] == "CSV"
        if haskey(data["settings"],"csvDirectory")
            if typeof(data["settings"]["csvDirectory"])==String
                # 12/15 old, not needed if already initialized: merge!(settings, Dict(:csvDirectory => data["settings"]["csvDirectory"]))
                settings[:csvDirectory] = data["settings"]["csvDirectory"]
            else
                error("ERROR at reading the settings yaml file: The value of csvDirectory is wrong. It has to be of type String.")
            end
        else
            error("ERROR at reading the settings yaml file: The keyword csvDirectory is missing. It has to be added.")
        end
        delete!(data["settings"], "csvDirectory")
    end # if


    # should the output be "minimal" or "driving course"
    if haskey(data["settings"],"detailOfOutput")
        if typeof(data["settings"]["detailOfOutput"])==String && (data["settings"]["detailOfOutput"]=="minimal" || data["settings"]["detailOfOutput"]=="driving course")
            # 12/15 old, not needed if already initialized: merge!(settings, Dict(:detailOfOutput => data["settings"]["detailOfOutput"]))             # "minimal" or "driving course"
            settings[:detailOfOutput] = data["settings"]["detailOfOutput"]
        else
            error("ERROR at reading the settings yaml file: The value of detailOfOutput is wrong. It has to be minimal or driving course.")
        end
    else
        error("ERROR at reading the settings yaml file: The keyword detailOfOutput is missing. It has to be added with the value minimal or driving course.")
    end
    delete!(data["settings"], "detailOfOutput")


    # inform the user, which keywords of the input data are not used in this tool:
    if length(data["settings"])>0
        println("INFO at reading the settings yaml file: The following Keywords are not used in this tool:")
        for key in keys(data["settings"])
            println("  - ",key)
        end
    end

    return settings
end # function inputSettings

end # module Input
