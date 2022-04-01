#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

module Import

import YAML

export importYamlFiles, importYamlFile

@enum trainType passenger=1 freight=2 motorCoachTrain=3

"""
Read the input information from YAML files for train, path and settings, save it in different dictionaries and return them.
"""
function importYamlFiles(trainDirectory::String, pathDirectory::String, settingsDirectory::String)
     train = importTrainFromYaml(trainDirectory)
     path = importPathFromYaml(pathDirectory)
     settings = importSettingsFromYaml(settingsDirectory)

     return (train, path, settings)
end #function importYamlFiles

 """
 Read the input information from one of the YAML files for train, path or settings, save it in a Dictionary and return it.
 """
function importYamlFile(dataType::Symbol, directory::String)
    if dataType == :train
        return importTrainFromYaml(directory)
    elseif dataType == :path
        return importPathFromYaml(directory)
    elseif dataType == :settings
        return importSettingsFromYaml(directory)
    else
        error("Wrong dataType in function importYamlFile")
    end
 end #function importYamlFile

"""
Read the train information from a YAML file, save it in a train Dict and return it.
"""
function importTrainFromYaml(trainDirectory::String)
    data = YAML.load(open(trainDirectory))

    name = getString!(data, "train", "name")    # train's name
    id=1                                        # train's identifier
    type = getTrainType!(data)                  # "passenger" or "freight" or "motorCoachTrain"

    trainLength = getPositiveNumber!(data, "train", "l_train", "m", true)   # total length (in m)
    v_limit = getSpeedLimit!(data)                                          # train's speed limit (in m/s)
    a_braking = get_a_braking!(data)                                        # a_braking

    m_td = getPositiveNumber!(data, "train", "m_td", "kg", true)            # mass on the traction unit's driving axles (in kg)
    m_tc = getPositiveNumber!(data, "train", "m_tc", "kg", false)           # mass on the traction unit's carrying axles (in kg)
    m_t=m_td+m_tc                                                           # mass of the traction unit (in kg)
    m_w = getPositiveNumber!(data, "train", "m_w", "kg", false)             # mass of the set of wagons (consist)  (in kg)
    m_train=m_t+m_w                                                         # total mass (in kg)

    (ξ_train, ξ_t, ξ_w) = getRotationMassFactors!(data, m_train, m_t, m_w)

    tractiveEffortVelocityPairs = getTractiveEffortVelocityPairs!(data)  # pairs of velocity and tractive effort

    # coefficients for the vehicle resistance of the traction unit
    Δv_t=15.0/3.6                                                        # coefficient for velocitiy difference between traction unit and outdoor air (in m/s)
    f_Rtd0 = getPositiveNumber!(data, "train", "f_Rtd0", "‰", false)     # coefficient for basic resistance due to the traction units driving axles (in ‰)
    f_Rtc0 = getPositiveNumber!(data, "train", "f_Rtc0", "‰", false)     # coefficient for basic resistance due to the traction units carring axles (in ‰)
    F_Rt2 = getPositiveNumber!(data, "train", "F_Rt2", "N", false)       # coefficient for air resistance of the traction units (in N)



    # coefficients for the vehicle resistance of the set of wagons (consist)
    Δv_w = get_Δv_w!(type)                                       # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)
    f_Rw0 = getPositiveNumber!(data, "train", "f_Rw0", "‰", false)       # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    f_Rw1 = getPositiveNumber!(data, "train", "f_Rw1", "‰", false)       # coefficient for the consists resistance to rolling (in ‰)
    f_Rw2 = getPositiveNumber!(data, "train", "f_Rw2", "‰", false)       # coefficient fo the consistsr air resistance (in ‰)

    informAboutUnusedKeys(data, "train") # inform the user, which keywords of the imported data are not used in this tool

    # create the train Dictionary
    train= Dict(:name => name,              # train's name
                :id => id,                  # train's identifier
                :type => type,              # type of train "passenger" or "freight" or "motorCoachTrain"
            #= 01/05 old without enum   :type => type,    # type of train "passenger" or "freight" or "motor coach train" =#
                :length => trainLength,     # total length (in m)
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
                :Δv_t => Δv_t,          # coefficient for velocitiy difference between traction unit and outdoor air (in m/s)

                # set of wagons
                :m_w => m_w,            # mass of the set of wagons (in kg)
                :ξ_w => ξ_w,            # rotation mass factor of the set of wagons (without unit)
                                        # in case ξ_train is not available
                :f_Rw0 => f_Rw0,        # coefficient for basic resistance of the set of wagons (in ‰)
                :f_Rw1 => f_Rw1,        # coefficient for resistance to rolling of the set of wagons (in ‰)
                :f_Rw2 => f_Rw2,        # coefficient for air resistance of the set of wagons (in ‰)
                :Δv_w => Δv_w)          # coefficient for velocitiy difference between set of wagons and outdoor air (in m/s)

    return train
end #function importTrainFromYaml

function importPathFromYaml(pathDirectory::String)
 # read path information from a YAML file, save it in a path Dict and return it
    data = YAML.load(open(pathDirectory))

    name = getString!(data, "path", "name")
    id=1                                     # path identifier
    sections = getSections!(data)

    # save values in the path Dict
    path = Dict(:name => name,
                :id => id,
                :sections => sections)

    addPointsOfInterest!(path, data)

    informAboutUnusedKeys(data, "path") # inform the user, which keywords of the imported data are not used in this tool

    return path
end # function importPathFromYaml


## settings for the calculation
function importSettingsFromYaml(settingsDirectory::String)
 # read setting information from a YAML file, save it in a settings Dict and return it
    data = YAML.load(open(settingsDirectory))

    # initialize the settings Dictionary
    settings = Dict(:massModel => "",                                   # model type of the train's mass "mass point" or "homogeneous strip"
                    :stepVariable => "",                                # step variable of the step method "s in m", "t in s" or "v in m/s"
                    :stepSize => 0.0,                                   # step size (unit depends on stepVariable s in m, t in s and v in m/s)
                    :operationModeMinimumRunningTime => false,          # operation mode "minimum running time"
                    :operationModeMinimumEnergyConsumption => false,    # operation mode "minimum energy consumption"
                    :typeOfOutput => "",                                # output as "julia dictionary" or as "CSV"
                    :csvDirectory => "",                                # path of the folder in which the CSV files willl be saved
                    :detailOfOutput => "")                              # detail of output "minimal" or "everything"

    settings[:massModel] = getString!(data, "settings", "massModel", ["mass point", "homogeneous strip"])                   # model type of the train's mass "mass point" or "homogeneous strip"
    settings[:stepVariable] = getString!(data, "settings", "stepVariable", ["s in m", "t in s", "v in m/s"])                # step variable of the step method "s in m", "t in s" or "v in m/s"
    settings[:stepSize] = getPositiveNumber!(data, "settings", "stepSize", "("*settings[:stepVariable]*")", true)           # step size (unit depends on stepVariable: s in m, t in s and v in m/s)
    settings[:operationModeMinimumRunningTime] = getBool!(data, "settings", "operationModeMinimumRunningTime")              # operation mode "minimum running time"
    settings[:operationModeMinimumEnergyConsumption] = getBool!(data, "settings", "operationModeMinimumEnergyConsumption")  # operation mode "minimum energy consumption"
    settings[:typeOfOutput] = getString!(data, "settings", "typeOfOutput", ["julia dictionary", "CSV"])                     # output as "julia dictionary" or as "CSV"

    if settings[:typeOfOutput] == "CSV"
        settings[:csvDirectory] = getString!(data, "settings", "csvDirectory")
        # TODO: it could be checked if the path is existing on the pc
    end # if

    settings[:detailOfOutput] = getString!(data, "settings", "detailOfOutput", ["running time", "points of interest", "driving course", "everything"])   # should the output be only the value of the "running time", or an array of "points of interest" or the complete "driving course" as array or a dictionary with "everything"?
    # 30/31 old:     settings[:detailOfOutput] = getString!(data, "settings", "detailOfOutput", ["minimal", "points of interest", "driving course"])    # should the output be "minimal" or are "points of interest" or the complete "driving course" required?

    informAboutUnusedKeys(data, "settings")         # inform the user, which keywords of the imported data are not used in this tool

    return settings
end # function importSettingsFromYaml

function getBool!(data::Dict, dataSet::String, key::String)
    if haskey(data[dataSet],key) && data[dataSet][key]!=nothing
        if typeof(data[dataSet][key])==Bool
            value = data[dataSet][key]
        else
            error("ERROR at reading the ",dataSet," yaml file: The value of the keyword ",key," is not correct. The value has to be of type Bool.")
        end
    else
        value=false
        println("WARNING at reading the ",dataSet," yaml file: The keyword ",key," or its value is missing. Therefore ",key,"=",value," is assumed and used.")
    end
    delete!(data[dataSet], key)
    return value
end #function getBool!

function getPositiveNumber!(data::Dict, dataSet::String, key::String, unit::String, required::Bool)
    if haskey(data[dataSet],key) && data[dataSet][key]!=nothing
        if typeof(data[dataSet][key]) <: Real && data[dataSet][key] >= 0.0
            value = data[dataSet][key]
        else
            error("ERROR at reading the ",dataSet," yaml file: The value of ",key," is no real floating point number >=0.0.")
        end
    elseif required
        error("ERROR at reading the ",dataSet," yaml file: The keyword ",key," is missing. It has to be added with a value of type real floating point number >0.0.")
    else
        value=0.0
        println("WARNING at reading the ",dataSet," yaml file: The keyword ",key," is missing. Therefore ",key,"=0.0 ",unit," will be assumed and used." )
    end
    delete!(data[dataSet], key)

    return value
end #function getPositiveNumber!

function getString!(data::Dict, dataSet::String, key::String, validValues::Vector{String})
    # TODO change getString! to getSymbol! or to getEnum! ?
    if haskey(data[dataSet],key) && data[dataSet][key]!=nothing
        value = data[dataSet][key]
        if typeof(value)==String
            for validValue in validValues
                if value == validValue
                    delete!(data[dataSet], key)
                    return value
                end
            end
        end
        error("ERROR at reading the ",dataSet," yaml file: The value of ",key," is wrong. It has to be one of the following String values: ", validValues)
    else
        error("ERROR at reading the ",dataSet," yaml file: The keyword ",key," is missing. It has to be added with one of the following String values: ", validValues)
    end
end #function getString!
# second method of function getString! without validValues
function getString!(data::Dict, dataSet::String, key::String)
    if haskey(data[dataSet],key) && data[dataSet][key]!=nothing
        value = data[dataSet][key]
        if typeof(value)==String
            delete!(data[dataSet], key)
            return value
        end
        error("ERROR at reading the ",dataSet," yaml file: The value of ",key," is wrong. It has to be of type String.")
    else
        error("ERROR at reading the ",dataSet," yaml file: The keyword ",key," is missing. It has to be added.")
    end
end #function getString!

function getTrainType!(data::Dict)
    if haskey(data["train"],"trainType") && data["train"]["trainType"]!=nothing
        if typeof(data["train"]["trainType"])==String && (data["train"]["trainType"]=="freight" || data["train"]["trainType"]=="motorCoachTrain" || data["train"]["trainType"]=="passenger")
            # 01/05 old without enum: trainType=data["train"]["trainType"]                  # "passenger" or "freight" or "motorCoachTrain"
            type = getEnum(data["train"]["trainType"], trainType)       # "passenger" or "freight" or "motorCoachTrain"
            delete!(data["train"], "trainType")
        else
            error("ERROR at reading the train yaml file: The value of trainType is wrong. It has to be freight, motorCoachTrain or passenger.")
        end
    else
        error("ERROR at reading the train yaml file: The keyword trainType is missing. It has to be added with the value freight, motorCoachTrain or passenger.")
    end
    return type
end # function getTrainType!

function getSpeedLimit!(data::Dict)   # train's speed limit (in m/s)
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
        v_limit=1000.0/3.6
        println("WARNING at reading the train yaml file: There is no value for the trains speed limit (v_limit or v_limit_kmh). The value v_limit=1000 km/h =",v_limit," m/s is used." )
    end

    return v_limit
end #function getSpeedLimit!

function get_a_braking!(data)
    if haskey(data["train"],"a_braking") && data["train"]["a_braking"]!=nothing
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

    return a_braking
end #function get_a_braking!

function getRotationMassFactors!(data::Dict, m_train::Real, m_t::Real, m_w::Real)
    if haskey(data["train"],"rotationMassFactor_train") && data["train"]["rotationMassFactor_train"]!=nothing && typeof(data["train"]["rotationMassFactor_train"]) <: Real
        if data["train"]["rotationMassFactor_train"]>0.0
            ξ_train=data["train"]["rotationMassFactor_train"]
            ξ_t=0.0
            ξ_w=0.0
        else
            error("ERROR at reading the train yaml file: The value of rotationMassFactor_train is no real floating point number >0.0.")
        end
    elseif haskey(data["train"],"rotationMassFactor_t") && data["train"]["rotationMassFactor_t"]!=nothing  && typeof(data["train"]["rotationMassFactor_t"]) <: Real && (m_w==0.0 || (haskey(data["train"],"rotationMassFactor_w") && data["train"]["rotationMassFactor_w"]!=nothing && typeof(data["train"]["rotationMassFactor_w"]) <: Real))
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

    return (ξ_train, ξ_t, ξ_w)
end #function getRotationMassFactors!

function getTractiveEffortVelocityPairs!(data::Dict)  # pairs of velocity and tractive effort
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

    return tractiveEffortVelocityPairs
end #function getTractiveEffortVelocityPairs!

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

function get_Δv_w!(type::trainType)     # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)
    if type == passenger::trainType || type == motorCoachTrain::trainType
        Δv_w=15.0/3.6
    elseif type == freight::trainType
        Δv_w=0.0
    end # if

    #= 01/05 old without enum
    if type=="passenger" || type=="motor coach train"
        Δv_w=15.0/3.6
    elseif type== "freight"
        Δv_w=0.0
    end # if    =#

    return Δv_w
end #function get_Δv_w!

function getSections!(data::Dict)
    # read the section starting positions and corresponding information
    if haskey(data["path"],"sectionStarts") && data["path"]["sectionStarts"]!=nothing
        sectionStartsArray=data["path"]["sectionStarts"]
        conversionFactor=1.0                                # conversion factor between the units m/s and m/s
        valueKey="sectionStarts" 	                   # necessary for error messages

        if haskey(data["path"],"sectionStarts") && data["path"]["sectionStarts_kmh"]!=nothing
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

    # check if the array is correct and if elements of the array have the correct type and valid values
    errorDetected=false
    if length(sectionStartsArray)<2
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


    sections=[]
    for row in 2:length(sectionStartsArray)
        s_start=sectionStartsArray[row-1][1]                    # first point of the section (in m)
        s_end=sectionStartsArray[row][1]                        # first point of the next section (in m)
        v_limit=sectionStartsArray[row-1][2]*conversionFactor   # paths speed limt (in m/s)
        f_Rp=sectionStartsArray[row-1][3]                       # specific path resistance of the section (in ‰)
        section = Dict(:s_start => s_start,
                        :s_end => s_end,
                        :v_limit => v_limit,
                        :f_Rp => f_Rp)
        push!(sections, section)
    end # for
    # s_start in first entry defines the path's beginning
    # s_end in last entry defines the path's ending

    return sections
end #function getSections!

function addPointsOfInterest!(path::Dict, data::Dict)
    # read the section starting positions and corresponding information
    if haskey(data["path"],"pointsOfInterest") && data["path"]["pointsOfInterest"]!=nothing
        pointsOfInterest = data["path"]["pointsOfInterest"]
        delete!(data["path"], "pointsOfInterest")

        sortingNeeded = false
        errorDetected = false
        for element in 1:length(pointsOfInterest)
            if typeof(pointsOfInterest[element]) <: Real
                if element > 1
                    if pointsOfInterest[element] < pointsOfInterest[element-1]
                        sortingNeeded = true
                        println("INFO at reading the path yaml file: The point of interest in element ", element ," (",pointsOfInterest[element]," m) has to be higher than the value of the previous element (",pointsOfInterest[element-1]," m). The points of interest will be sorted.")
                    end
                end
            else
                errorDetected = true
                println("ERROR at reading the path yaml file: The point of interest in element ", element ," is no real floating point number.")
            end
        end # for

        if errorDetected
            error("ERROR at reading the path yaml file: The values of the point of interest have to be corrected.")
        end
        if sortingNeeded == true
            sort!(pointsOfInterest)
        end

        copiedPOIs = []
        for element in 1:length(pointsOfInterest)
            if element == 1
                push!(copiedPOIs, pointsOfInterest[element])
            elseif element > 1 && pointsOfInterest[element] > pointsOfInterest[element-1]
                push!(copiedPOIs, pointsOfInterest[element])
            end
        end # for
        merge!(path, Dict(:pointsOfInterest => copiedPOIs))
    end
    return (path, data)
end #function addPointsOfInterest!

function informAboutUnusedKeys(data::Dict, dataSet::String)         # inform the user which keywords of the imported data are not used in this tool
    if length(data[dataSet])>0
        println("INFO at reading the ",dataSet," yaml file: The following Keywords are not used in this tool:")
        for key in keys(data[dataSet])
            println("  - ",key)
        end
    end
end #function informAboutUnusedKeys

"""
    getEnum(string, enum_type)

Converts a string to an enumerated type.
But only if the string matches an enumerated value.

# Example
```jldoctest
julia> @enum trainType passenger freight

julia> myTrain = "passenger"
"passenger"

julia> myTrainType = getEnum(myTrain, trainType)
passenger::trainType = 0
```
"""
function getEnum(string::String, enum_type::DataType)
    inst   = instances(enum_type)  # get all instances of enumerated type
    syms   = Symbol.(inst)         # convert all instances to Symbols
    lookup = Dict(zip(syms, inst)) # combine instances and Symbols in a lookup table
    #
    n_str  = Symbol(string)        # normalize String via a Symbol
    return lookup[n_str]           # return matched enumerated type
end # function getEnum

end # module Input
