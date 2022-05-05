#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg, Martin Scheidt"
# __copyright__     = "2022"
# __license__       = "ISC"

"""
    Settings(file)

Settings is a datastruture for calculation context.
The function Settings() will create a set of settings for the train run calculation.
`file` is optinal may be used to load settings in the YAML format.

# Example
```jldoctest
julia> my_settings = Settings() # will generate default settings
Settings(mass_point, :distance, 20, 3, running_time, julia_dict, ".")
```
"""
struct Settings

    massModel::Symbol    # model type of train mass ":mass_point" or ":homogeneous_strip".
    stepVariable::Symbol # variable of the linear multistep method: ":distance", ":time" or ":velocity".
    stepSize::Real       # step size, unit depends on stepVariable - :distance in meter, time in seconds and velocity in meter/second.
    approxLevel::Int     # value for approximation; used when rounding or interating.
    outputDetail::Symbol # single Float() ":running_time", Array() of ":points_of_interest",
                         # complete Array() ":driving_course", or Dict() ":everything".
    outputFormat::Symbol # output as ":julia_dict" or as ":csv".
    outputDir::String    # if outputFormat is not ":julia_dict".

    ## constructor
    function Settings(file="DEFAULT")

        ## default values
        massModel    = :mass_point
        stepVariable = :distance
        stepSize     = 20
        approxLevel  = 3
        outputDetail = :running_time
        outputFormat = :julia_dict
        outputDir    = "."

        ## load from file
        if file != "DEFAULT"

            ## JSON schema for YAML-file validation
            schema = Schema("""{
                "properties": {
                    "massModel": {
                        "description": "type of train model",
                        "type": "string",
                        "enum": [ "mass_point", "homogeneous_strip" ]
                    },
                    "stepVariable": {
                        "description": "variable of the linear multistep method",
                        "type": "string",
                        "enum": [ "distance", "time", "velocity" ]
                    },
                    "stepSize": {
                        "description": "step size acording to stepVariable",
                        "type": "number",
                        "exclusiveMinimum": 0
                    },
                    "outputDetail": {
                        "description": "Selecting the detail of the result",
                        "type": "string",
                        "enum": [ "running_time", "points_of_interest", "driving_course", "everything" ]
                    },
                    "outputFormat": {
                        "description": "Output format",
                        "type": "string",
                        "enum": [ "julia_dict", "csv" ]
                    },
                    "outputDir": {
                        "description": "Path for the CSV export",
                        "type": "string"
                    }
                }
            }""")

            settings = YAML.load(open(file))["settings"]
            
            ## validate the loaded file
            try
                validate(schema, settings)
            catch err
                println("Could not load settings file '$file'.\n Format is not recognized - using default as fall back.")
                settings = Dict()
            end

            ## set the variables if they exist in "settings"
            haskey(settings, "massModel")    ? massModel    = Symbol(settings["massModel"])    : nothing
            haskey(settings, "stepVariable") ? stepVariable = Symbol(settings["stepVariable"]) : nothing
            haskey(settings, "stepSize")     ? stepSize     =        settings["stepSize"]      : nothing
            haskey(settings, "approxLevel")  ? approxLevel  =        settings["approxLevel"]   : nothing
            haskey(settings, "outputDetail") ? outputDetail = Symbol(settings["outputDetail"]) : nothing
            haskey(settings, "outputFormat") ? outputFormat = Symbol(settings["outputFormat"]) : nothing
            haskey(settings, "outputDir")    ? outputDir    =        settings["outputDir"]     : nothing
        end

        new(massModel, stepVariable, stepSize, approxLevel, outputDetail, outputFormat, outputDir)

    end #function Settings() # constructor

end #struct Settings

"""
    Path(file, type = :YAML)

Path is a datastruture for calculation context.
The function Path() will create a running path for the train.
Supported formats are: railtoolkit/schema (2022.05)

# Example
```jldoctest
julia> my_path = Path("file.yaml") # will generate a path from a YAML file.
Path(variables)
```
"""
struct Path

    name::String     # a name or description of the path
    id::String       # a short string as identifier
    uuid::UUID       # a unique identifier
    poi::Vector      # a vector of triples with points along the path
    sections::Vector # a vector of the characteristic sections

    ## constructor
    function Path(file, type = :YAML)

        ## default values
        name      = ""
        id        = ""
        uuid      = UUIDs.uuid4()
        poi       = []
        sections  = []

        ## process flags
        POI_PRESENT = false

        ## load from file
        if type == :YAML

            data = YAML.load(open(file))
            if data["schema"] != "https://railtoolkit.org/schema/running-path.json"
                error("Could not load path file '$file'.\n
                       YAML format is not recognized. 
                       Currently supported: railtoolkit/schema/running-path (2022.05)")
            end
            if data["schema_schema"] != "2022.05"
                error("Could not load path file '$file'.\n
                       YAML format is not recognized. 
                       Currently supported: railtoolkit/schema/running-path (2022.05)")
            end

            ## JSON schema for YAML-file validation
            railtoolkit_schema = Schema("""{
                "required": [ "schema", "schema_version", "paths" ],
                "properties": {
                  "schema": {
                    "description": "Identifier of the schema",
                    "enum": [ "https://railtoolkit.org/schema/running-path.json" ]
                  },
                  "schema_version": {
                    "description": "Version of the schema",
                    "type": "string",
                    "pattern": "[2-9][0-9][0-9][0-9].[0-1][0-9]"
                  },
                  "paths": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                      "required": [ "name", "id", "characteristic_sections" ],
                      "type": "object",
                      "properties": {
                        "characteristic_sections": {
                          "description": "",
                          "type": "array",
                          "minItems": 2,
                          "uniqueItems": true,
                          "items": {
                            "type": "array",
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "",
                            "prefixItems": [
                              {
                                "description": "milage in meter",
                                "type": "number"
                              },
                              {
                                "description": "speed in kilometers per hour",
                                "type": "number",
                                "exclusiveMinimum": 0
                              },
                              {
                                "description": "resistance in permil",
                                "type": "number"
                              }
                            ]
                          }
                        },
                        "id": {
                          "description": "Identifier of the path",
                          "type": "string"
                        },
                        "name": {
                          "description": "Name of the path",
                          "type": "string"
                        },
                        "points_of_interest": {
                          "description": "",
                          "type": "array",
                          "uniqueItems": true,
                          "items": {
                            "type": "array",
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "",
                            "prefixItems": [
                              { "type": "number" },
                              { "type": "string" },
                              { "enum": [ "front", "rear" ] }
                            ]
                          }
                        },
                        "UUID": {
                          "description": "The unique identifier for a path",
                          "type": "string",
                          "format": "uuid"
                        }
                      }
                    }
                  }
                }
            }""")

            paths = data["paths"]
            try
                validate(railtoolkit_schema, paths)
            catch err
                error("Could not load path file '$file'.\n
                       YAML format is not recognized. 
                       Currently supported: railtoolkit/schema/running-path (2022.05)")
            end
            if length(paths) > 1
                println("WARNING: the loaded file contains more than one path. Using only the first!")
            end
            path = paths[1]

            ## set the variables if they exist in "settings"
            # required
            name    = path["name"]
            id      = path["id"]
            tmp_sec = path["characteristic_sections"]
            # optional
            haskey(path, "UUID")               ? uuid        = parse(UUID, path["UUID"] ) : nothing
            haskey(path, "points_of_interest") ? POI_PRESENT = true                       : nothing
            haskey(path, "points_of_interest") ? tmp_points  = path["points_of_interest"] : nothing

        else
            error("Unknown file type '$type'")
        end #if type

        ## process characteristic sections
        sort!(tmp_sec, by = x -> x[1])
        for row in 2:length(tmp_sec)
            s_start = tmp_sec[row-1][1]     # first point of the section (in m)
            s_end   = tmp_sec[row][1]       # first point of the next section (in m)
            v_limit = tmp_sec[row-1][2]/3.6 # paths speed limt (in m/s)
            f_Rp    = tmp_sec[row-1][3]     # specific path resistance of the section (in ‰)
    
            section = Dict(:s_start => s_start,
                           :s_end   => s_end,
                           :v_limit => v_limit,
                           :f_Rp    => f_Rp)
            push!(sections, section)
        end #for row
        # s_start in first entry defines the path's beginning
        # s_end in last entry defines the path's ending

        ## process points of interest
        if POI_PRESENT
            sort!(tmp_points, by = x -> x[1])
            for elem in tmp_points
                station = elem[1]     # first point of the section (in m)
                label   = elem[2] # paths speed limt (in m/s)
                measure = elem[3]     # specific path resistance of the section (in ‰)
        
                point = Dict(:station => station,
                             :label   => label,
                             :measure => measure)
                push!(poi, point)
            end #for elem
        end #if !isempty(points)

        new(name, id, uuid, poi, sections)

    end #function Path

end #struct Path

"""
a DataPoint is the smallest element of the driving course. One step of the step approach is between two data points
"""
function createDataPoint()
    dataPoint = Dict(
        :i => 0,            # identifier and counter variable of the driving course
        :behavior => "",    # type of behavior section the data point is part of - see createBehaviorSection()
                            # a data point which is the last point of one behavior section and the first point of the next behavior section will be attached to the latter
        :s => 0.0,          # position (in m)
        :Δs => 0.0,         # step size (in m)
        :t => 0.0,          # point in time (in s)
        :Δt => 0.0,         # step size (in s)
        :v => 0.0,          # velocity (in m/s)
        :Δv => 0.0,         # step size (in m/s)
        :a => 0.0,          # acceleration (in m/s^2)
        :W => 0.0,          # mechanical work (in Ws)
        :ΔW => 0.0,         # mechanical work in this step (in Ws)
        :E => 0.0,          # energy consumption (in Ws)
        :ΔE => 0.0,         # energy consumption in this step (in Ws)
        :F_T => 0.0,        # tractive effort (in N)
        :F_R => 0.0,        # resisting force (in N)
        :R_path => 0.0,     # path resistance (in N)
        :R_train => 0.0,    # train resistance (in N)
        :R_traction => 0.0, # traction unit resistance (in N)
        :R_wagons => 0.0,   # set of wagons resistance (in N)
        :label => ""        # a label for importend points
    )
    return dataPoint
end #function createDataPoint

"""
BehaviorSection() TODO!
"""
function createBehaviorSection(type::String, s_entry::Real, v_entry::Real, startingPoint::Integer)
    BS= Dict(
        :type => type,                 # type of behavior section: "breakFree", "clearing", "accelerating", "cruising", "downhillBraking", "diminishing", "coasting", "braking" or "standstill"
        :length => 0.0,                # total length  (in m)
        :s_entry => s_entry,           # first position (in m)
        :s_exit => 0.0,                # last position  (in m)
        :t => 0.0,                     # total running time (in s)
        :E => 0.0,                     # total energy consumption (in Ws)
        :v_entry => v_entry,           # entry speed (in m/s)
        :v_exit => 0.0,                # exit speed (in m/s)
        :dataPoints => [startingPoint] # list of identifiers of the containing data points starting with the initial point
    )
    return BS
end #function createBehaviorSection

## create a moving section containing characteristic sections
function createMovingSection(path::Path, v_trainLimit::Real, s_trainLength::Real)
    # this function creates and returns a moving section dependent on the paths attributes

    s_entry = path.sections[1][:s_start]          # first position (in m)
    s_exit = path.sections[end][:s_end]           # last position (in m)
    pathLength = s_exit - s_entry                   # total length (in m)

    CSs=Vector{Dict}()
    s_csStart=s_entry
    csId=1
    for row in 2:length(path.sections)
        previousSection = path.sections[row-1]
        currentSection = path.sections[row]
        speedLimitIsDifferent = min(previousSection[:v_limit], v_trainLimit) != min(currentSection[:v_limit], v_trainLimit)
        pathResistanceIsDifferent = previousSection[:f_Rp] != currentSection[:f_Rp]
        if speedLimitIsDifferent || pathResistanceIsDifferent
        # 03/09 old: if min(previousSection[:v_limit], v_trainLimit) != min(currentSection[:v_limit], v_trainLimit) || previousSection[:f_Rp] != currentSection[:f_Rp]
            push!(CSs, createCharacteristicSection(csId, s_csStart, previousSection, min(previousSection[:v_limit], v_trainLimit), s_trainLength, path))
            s_csStart = currentSection[:s_start]
            csId = csId+1
        end #if
    end #for
    push!(CSs, createCharacteristicSection(csId, s_csStart, path.sections[end], min(path.sections[end][:v_limit], v_trainLimit), s_trainLength, path))

    movingSection= Dict(:id => 1,                       # identifier    # if there is more than one moving section in a later version of this tool the id should not be constant anymore
                        :length => pathLength,          # total length (in m)
                        :s_entry => s_entry,            # first position (in m)
                        :s_exit => s_exit,              # last position (in m)
                        :t => 0.0,                      # total running time (in s)
                        :E => 0.0,                      # total energy consumption (in Ws)
                        :characteristicSections => CSs) # list of containing characteristic sections

    return movingSection
end #function createMovingSection


## create a characteristic section for a path section. A characteristic section is a part of the moving section. It contains behavior sections.
function createCharacteristicSection(id::Integer, s_entry::Real, section::Dict, v_limit::Real, s_trainLength::Real, path::Path)
    # Create and return a characteristic section dependent on the paths attributes
    characteristicSection= Dict(:id => id,                            # identifier
                                :s_entry => s_entry,                    # first position (in m)
                                :s_exit => section[:s_end],             # last position  (in m)
                                :length => section[:s_end] -s_entry,    # total length  (in m)
                                :r_path => section[:f_Rp],              # path resistance (in ‰)
                                :behaviorSections => Dict(),            # list of containing behavior sections
                                :t => 0.0,                              # total running time (in s)
                                :E => 0.0,                              # total energy consumption (in Ws)
                                :v_limit => v_limit,                    # speed limit (in m/s)
                                # initializing :v_entry, :v_peak and :v_exit with :v_limit
                                :v_peak => v_limit,                     # maximum reachable speed (in m/s)
                                :v_entry => v_limit,                    # maximum entry speed (in m/s)
                                :v_exit => v_limit)                     # maximum exit speed (in m/s)

    # list of positions of every point of interest (POI) in this charateristic section for which data points should be calculated
    s_exit = characteristicSection[:s_exit]

    ##TODO: use a tuple with naming
    pointsOfInterest = Tuple[]
    # pointsOfInterest = Real[]
    if !isempty(path.poi)
        for POI in path.poi
            s_poi = POI[:station]
            if POI[:measure] == "rear"
                s_poi -= s_trainLength
            end
            if s_entry < s_poi && s_poi < s_exit
                push!(pointsOfInterest, (s_poi, POI[:label]) )
                # push!(pointsOfInterest, s_poi )
            end
        end
    end
    push!(pointsOfInterest, (s_exit,""))     # s_exit has to be the last POI so that there will always be a POI to campare the current position with
    # push!(pointsOfInterest, s_exit)     # s_exit has to be the last POI so that there will always be a POI to campare the current position with

    merge!(characteristicSection, Dict(:pointsOfInterest => pointsOfInterest))

    return characteristicSection
end #function createCharacteristicSection

"""
Read the train information from a YAML file, save it in a train Dict and return it.
"""
function checkAndSetTrain!(train::Dict)
 # check train information from input dictionary

    checkAndSetString!(train, "train", :name, "")    # train's name
    # add train's identifier if not existing
    if !(haskey(train, :id) && train[:id]!=nothing)
        merge!(train, Dict(:id =>1))
    end
    checkAndSetString!(train, "train", :type, "passenger", ["passenger", "freight"])      # train type "passenger" or "freight"

    checkAndSetPositiveNumberWithDifferentNames!(train, "train", :length, :l_train, "m", 20.0)  # total length (in m)
    # TODO: or just use:     checkAndSetPositiveNumber!(train, "train", :length, "m", 20.0)

    checkAndSetSpeedLimit!(train)                           # train's speed limit (in m/s)
    checkAndSetBrakingAcceleration!(train)                          # a_braking

    checkAndSetPositiveNumber!(train, "train", :m_td, "kg", 80000)            # mass on the traction unit's driving axles (in kg)
    checkAndSetPositiveNumber!(train, "train", :m_tc, "kg", 0.0)     # mass on the traction unit's carrying axles (in kg)
    checkAndSetSum!(train, "train", :m_t, :m_td, :m_tc)         # mass of the traction unit (in kg)

    checkAndSetPositiveNumber!(train, "train", :m_w, "kg", 0.0)      # mass of the set of wagons (consist)  (in kg)
    checkAndSetSum!(train, "train", :m_train, :m_t, :m_w)       # total mass (in kg)
    if train[:m_train] <= 0.0
        error("ERROR at checking the input for the train: The train's mass has to be higher than 0.0 kg.")
    end

    checkAndSetRotationMassFactors!(train)
    checkAndSetTractiveEffortVelocityPairs!(train)  # pairs of velocity and tractive effort

    # coefficients for the vehicle resistance of the traction unit
    checkAndSetRealNumber!(train, "train", :Δv_t, "m/s", 15.0/3.6)            # coefficient for velocitiy difference between traction unit and outdoor air (in m/s)
    checkAndSetPositiveNumber!(train, "train", :f_Rtd0, "‰", 0.0)   # coefficient for basic resistance due to the traction units driving axles (in ‰)
    checkAndSetPositiveNumber!(train, "train", :f_Rtc0, "‰", 0.0)   # coefficient for basic resistance due to the traction units carring axles (in ‰)
    checkAndSetPositiveNumber!(train, "train", :F_Rt2, "N", 0.0)    # coefficient for air resistance of the traction units (in N)

    # coefficients for the vehicle resistance of the set of wagons (consist)
    checkAndSetRealNumber!(train, "train", :Δv_w, "m/s", getDefault_Δv_w(train[:type]))            # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)
    checkAndSetPositiveNumber!(train, "train", :f_Rw0, "‰", 0.0)    # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    checkAndSetPositiveNumber!(train, "train", :f_Rw1, "‰", 0.0)    # coefficient for the consists resistance to rolling (in ‰)
    checkAndSetPositiveNumber!(train, "train", :f_Rw2, "‰", 0.0)    # coefficient fo the consistsr air resistance (in ‰)

    # inform the user about keys of the input dictionary that are not used in this tool
    usedKeys = [:name, :id, :type,
                :length, :l_train, :v_limit, :v_limit_kmh, :a_braking,
                :m_train, :m_t, :m_td, :m_tc, :m_w,
                :ξ_train, :ξ_t, :ξ_w, :rotationMassFactor_train, :rotationMassFactor_t, :rotationMassFactor_w,
                :tractiveEffortVelocityPairs, :F_T_pairs, :F_T_pairs_kmh,
                :f_Rtd0, :f_Rtc0, :F_Rt2, :Δv_t,
                :f_Rw0, :f_Rw1, :f_Rw2, :Δv_w]
    informAboutUnusedKeys(collect(keys(train)), usedKeys::Vector{Symbol}, "train")

    return train
end #function checkAndSetTrain!

function checkAndSetPath!(path::Path)
 # check path information from input dictionary

    checkAndSetString!(path, "path", :name, "")
    # TODO checkId ? path[:id]                                                  # path identifier
    checkAndSetSections!(path)
    checkAndSetPOIs!(path)

    # inform the user about keys of the input dictionary that are not used in this tool
    usedKeys = [:name,
                :sections, :sectionStarts, :sectionStarts_kmh,
                :pointsOfInterest]
    informAboutUnusedKeys(collect(keys(path)), usedKeys::Vector{Symbol}, "path")

    return path
end # function checkAndSetPath!

function checkAndSetBool!(dictionary::Dict, dictionaryType::String, key::Symbol, defaultValue::Bool)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key]) != Bool
            error("ERROR at checking the input for the ",dictionaryType,": The value of the key ",String(key)," is not correct. The value has to be of type Bool.")
        end
    else
        merge!(dictionary, Dict(key => defaultValue))
        defaultValue && println("INFO at checking the input for the ",dictionaryType,": The key ",String(key)," or its value is missing. Therefore ",String(key),"=",dictionary[key]," is assumed and used.")
    end
    return dictionary
end #function checkAndSetBool!

function checkAndSetPositiveNumber!(dictionary::Dict, dictionaryType::String, key::Symbol, unit::String, default::Real)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key]) <: Real && dictionary[key] >= 0.0
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",String(key)," is no real floating point number >=0.0.")
        end
    else
        merge!(dictionary, Dict(key => default))
        println("INFO at checking the input for the ",dictionaryType,": The key ",String(key)," is missing. Therefore ",String(key),"=",default," ",unit," will be assumed and used." )
    end

    return dictionary
end #function checkAndSetPositiveNumber!

# first method without a default value
function checkAndSetPositiveNumberWithDifferentNames!(dictionary::Dict, dictionaryType::String, mainKey::Symbol, alternativeKey::Symbol, unit::String)
    mainKey_temp = -1.0
    alternativeKey_temp = -1.0

    if haskey(dictionary, mainKey) && dictionary[mainKey]!=nothing
        if typeof(dictionary[mainKey]) <: Real && dictionary[mainKey] >= 0.0
            mainKey_temp = dictionary[mainKey]
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",mainKey," is no real floating point number >=0.0.")
        end
    end

    if haskey(dictionary, alternativeKey) && dictionary[alternativeKey]!=nothing
        if typeof(dictionary[alternativeKey]) <: Real && dictionary[alternativeKey] >= 0.0
            alternativeKey_temp = dictionary[alternativeKey]
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",alternativeKey," is no real floating point number >=0.0.")
        end
    else
        delete!(dictionary, alternativeKey)
    end

    if mainKey_temp >= 0.0 && alternativeKey_temp >= 0.0
        difference = abs(mainKey_temp - alternativeKey_temp)
        if difference > 1/(10^approxLevel)       # TODO or use difference > 0.0 ?
            delete!(dictionary, alternativeKey)
            println("WARNING at checking the input for the ",dictionaryType,": The values of ",mainKey," and ",alternativeKey," differ by ",difference," ",unit,". The value ",String(mainKey),"=",default," ",unit," is used." )
        end
    elseif mainKey_temp >= 0.0
        # do nothing
    elseif alternativeKey_temp >= 0.0
        merge!(dictionary, Dict(mainKey => alternativeKey_temp))
    else
        # do nothing
    end

    return dictionary
end #function checkAndSetPositiveNumberWithDifferentNames!

# second method with a default value
function checkAndSetPositiveNumberWithDifferentNames!(dictionary::Dict, dictionaryType::String, mainKey::Symbol, alternativeKey::Symbol, unit::String, default::Real)
    mainKey_temp = -1.0
    alternativeKey_temp = -1.0

    if haskey(dictionary, mainKey) && dictionary[mainKey]!=nothing
        if typeof(dictionary[mainKey]) <: Real && dictionary[mainKey] >= 0.0
            mainKey_temp = dictionary[mainKey]
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",mainKey," is no real floating point number >=0.0.")
        end
    end

    if haskey(dictionary, alternativeKey) && dictionary[alternativeKey]!=nothing
        if typeof(dictionary[alternativeKey]) <: Real && dictionary[alternativeKey] >= 0.0
            alternativeKey_temp = dictionary[alternativeKey]
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",alternativeKey," is no real floating point number >=0.0.")
        end
    else
        delete!(dictionary, alternativeKey)
    end

    if mainKey_temp >= 0.0 && alternativeKey_temp >= 0.0
        difference = abs(mainKey_temp - alternativeKey_temp)
        if difference > 1/(10^approxLevel)       # TODO or use difference > 0.0 ?
            delete!(dictionary, alternativeKey)
            println("WARNING at checking the input for the ",dictionaryType,": The values of ",mainKey," and ",alternativeKey," differ by ",difference," ",unit,". The value ",String(mainKey),"=",default," ",unit," is used." )
        end
    elseif mainKey_temp >= 0.0
        # do nothing
    elseif alternativeKey_temp >= 0.0
        merge!(dictionary, Dict(mainKey => alternativeKey_temp))
    else
        # set a default value
        merge!(dictionary, Dict(mainKey, default))
        println("INFO at checking the input for the ",dictionaryType,": The key ",mainKey," or its value is missing. Therefore the value ",String(mainKey),"=",default," ",unit," is used." )
    end

    return dictionary
end #function checkAndSetPositiveNumberWithDifferentNames!

function checkAndSetRealNumber!(dictionary::Dict, dictionaryType::String, key::Symbol, unit::String, default::Real)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key]) <: Real
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",String(key)," is no real number.")
        end
    else
        merge!(dictionary, Dict(key => default))
        println("INFO at checking the input for the ",dictionaryType,": The key ",String(key)," is missing. Therefore ",String(key),"=",default," ",unit," will be assumed and used." )
    end

    return dictionary
end #function checkAndSetRealNumber!

function checkAndSetSum!(dictionary::Dict, dictionaryType::String, sum::Symbol, summand1::Symbol, summand2::Symbol)
    if haskey(dictionary,sum) && dictionary[sum]!=nothing
        if typeof(dictionary[sum]) <: Real && dictionary[sum] >= 0.0
            difference = abs(dictionary[sum] - (dictionary[summand1]+dictionary[summand2]))
            if difference > 1/(10^approxLevel)
                error("ERROR at checking the input for the ",dictionaryType,": The value of ",String(sum)," is not exactly the sum of ",String(summand1)," and ",String(summand2),". It differs by ",difference,".")
            end
        else
            error("ERROR at checking the input for the ",dictionaryType,": The value of ",String(sum)," is no real floating point number >=0.0.")
        end
    else
        merge!(dictionary, Dict(sum => dictionary[summand1]+dictionary[summand2]))
        println("INFO at checking the input for the ",dictionaryType,": The key ",String(sum)," is missing. Therefore ",String(sum)," = ",String(summand1)," + ",String(summand2)," = ",dictionary[sum]," was calculated and will be used." )
    end

    return dictionary
end #function checkAndSetSum!

function checkAndSetString!(dictionary::Dict, dictionaryType::String, key::Symbol, defaultValue::String, validValues::Vector{String})
    # TODO change checkAndAddString! to checkAndAddSymbol!  ?
    if haskey(dictionary,key) && dictionary[key]!=nothing
        value = dictionary[key]
        if typeof(value) == String
            for validValue in validValues
                if value == validValue
                    return dictionary
                end
            end
        end
        error("ERROR at checking the input for the ",dictionaryType,": The value of ",String(key)," is wrong. It has to be one of the following String values: ", validValues)
    else
        println("INFO at checking the input for the ",dictionaryType,": The key ",String(key)," is missing. It has to be one of the following String values: ", validValues,". For this calculation the default value '",defaultValue,"' will be used.")
        merge!(dictionary, Dict(key => defaultValue))
    end
    return dictionary
end #function checkAndSetString!
# second method of function checkAndSetString! without validValues
function checkAndSetString!(dictionary::Dict, dictionaryType::String, key::Symbol, defaultValue::String)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        value = dictionary[key]
        if typeof(value) == String
            return dictionary
        end
        error("ERROR at checking the input for the ",dictionaryType,": The value of ",String(key)," is wrong. It has to be of type String.")
    else
        println("INFO at checking the input for the ",dictionaryType,": The key ",String(key)," is missing. For this calculation the default value '",defaultValue,"' will be used.")
        merge!(dictionary, Dict(key => defaultValue))
    end
    return dictionary
end #function checkAndSetString!

function checkAndSetSpeedLimit!(train::Dict)
    v_limit_temp = 0.0
    v_limit_kmh_temp = 0.0

    if haskey(train, :v_limit) && train[:v_limit]!=nothing
        if typeof(train[:v_limit]) <: Real && train[:v_limit] >= 0.0
            v_limit_temp = train[:v_limit]
        else
            error("ERROR at checking the input for the train: The value of v_limit is no real floating point number >=0.0.")
        end
    end

    if haskey(train, :v_limit_kmh) && train[:v_limit_kmh]!=nothing
        if typeof(train[:v_limit_kmh]) <: Real && train[:v_limit_kmh] >= 0.0
            v_limit_kmh_temp = train[:v_limit_kmh]
        else
            error("ERROR at checking the input for the train: The value of v_limit_kmh is no real floating point number >=0.0.")
        end
    else
        delete!(train, :v_limit_kmh)
    end

    if v_limit_temp > 0.0 && v_limit_kmh_temp > 0.0
        difference = abs(v_limit_temp - v_limit_kmh_temp/3.6)
        if difference > 1/(10^approxLevel)       # TODO or use difference > 0.0 ?
            delete!(train, :v_limit_kmh)
            println("WARNING at checking the input for the train: The values of v_limit and v_limit_kmh differ by ",difference," m/s. The value v_limit=",v_limit_temp," m/s is used." )
        end
    elseif v_limit_temp > 0.0
        # do nothing
    elseif v_limit_kmh_temp > 0.0
        merge!(train, Dict(:v_limit => v_limit_kmh_temp/3.6))
    else
        # set a default value
        merge!(train, Dict(:v_limit, 1000.0/3.6))           # set speed limit to 1000 km/h
        println("INFO at checking the input for the train: There is no value for the trains speed limit (v_limit or v_limit_kmh). The value v_limit=1000 km/h =",train[:v_limit]," m/s is used." )
    end

    return train
end #function checkAndSetSpeedLimit!

function checkAndSetBrakingAcceleration!(train::Dict)
    if haskey(train, :a_braking) && train[:a_braking]!=nothing
        if typeof(train[:a_braking]) <: Real
            if train[:a_braking] > 0.0
                train[:a_braking] =-train[:a_braking]
                println("INFO at checking the input for the train: The value for a_braking is >0.0. The braking acceleration has to be <0.0. Therefore a_braking=",train[:a_braking]," m/s^2 is used." )
            elseif train[:a_braking] == 0.0
                error("ERROR at checking the input for the train: The value for a_braking is 0.0. The braking acceleration has to be <0.0.")
            end
        else
            error("ERROR at checking the input for the train: The value for a_braking is no real floating point number <0.0.")
        end
    else
        # set a default value depending on the train type
        if train[:type] == "freight"
            a_braking = -0.225
        elseif train[:type] == "passenger"
            a_braking = -0.375
        #elseif train[:type] == "passengerSuburban"
        #    a_braking = -0.525
        # TODO: add suburban trains to train type?
        end

        merge!(train, Dict(:a_braking => a_braking))
        println("INFO at checking the input for the train: The key for a_braking is missing. Because of the train type ",train[:type]," a_braking=",a_braking," m/s^2 will be assumed and used." )
    end

    return train
end #function checkAndSetBrakingAcceleration!

function checkAndSetRotationMassFactors!(train::Dict)
    checkAndSetPositiveNumberWithDifferentNames!(train, "train", :ξ_train, :rotationMassFactor_train, "")
    checkAndSetPositiveNumberWithDifferentNames!(train, "train", :ξ_t, :rotationMassFactor_t, "")
    checkAndSetPositiveNumberWithDifferentNames!(train, "train", :ξ_w, :rotationMassFactor_w, "")
    if haskey(train, :ξ_train) && train[:ξ_train]!=nothing
        if train[:ξ_train]>0.0
            if haskey(train, :ξ_t) && train[:ξ_t]!=nothing && train[:ξ_t]>0.0 && (train[:m_w]==0.0 || (haskey(train, :ξ_w) && train[:ξ_w]!=nothing))
                # TODO: is && train[:ξ_t]>0.0 necessary here?
                difference = abs(train[:ξ_train] - (train[:ξ_t]*train[:m_t] + train[:ξ_w]*train[:m_w])/train[:m_train])
                if difference > 1/(10^approxLevel)
                    error("ERROR at checking the input for the train: The value of ξ_train is not exactly ξ_train=(ξ_t*m_t + ξ_w*m_w)/m_train. It differs by ",difference,".")
                end
            end
        else
            error("ERROR at checking the input for the train: The value of :ξ_train is no real floating point number >0.0.")
        end
    else
        checkAndSetPositiveNumber!(train, "train", :ξ_t, "", 1.09)

        if train[:m_w]>0.0
            default_ξ_w = 1.06
        else
            default_ξ_w = 0.0
        end
        checkAndSetPositiveNumber!(train, "train", :ξ_w, "", default_ξ_w)


        ξ_train=(train[:ξ_t]*train[:m_t] + train[:ξ_w]*train[:m_w])/train[:m_train]  # rotation mass factor of the whole train (without unit)
        if ξ_train <= 0.0
            error("ERROR at checking the input for the train: The train's rotations mass factor has to be higher than 0.0 kg.")
        end
        merge!(train, Dict(:ξ_train => ξ_train))
    end

    return train
end #function checkAndSetRotationMassFactors!

function checkAndSetTractiveEffortVelocityPairs!(train::Dict)  # pairs of velocity and tractive effort
    if haskey(train,:tractiveEffortVelocityPairs) && train[:tractiveEffortVelocityPairs]!=nothing
        pairs = train[:tractiveEffortVelocityPairs]
        velocityMultiplier = 1.0

        if (haskey(train,:F_T_pairs) && train[:F_T_pairs]!=nothing) && (haskey(train,:F_T_pairs_kmh) && train[:F_T_pairs_kmh]!=nothing)
            println("WARNING at checking the input for the train: There are values for tractiveEffortVelocityPairs, F_T_pairs and F_T_pairs_kmh. The values for tractiveEffortVelocityPairs are used." )

        elseif haskey(train,:F_T_pairs) && train[:F_T_pairs]!=nothing
            println("WARNING at checking the input for the train: There are values for tractiveEffortVelocityPairs and F_T_pairs. The values for tractiveEffortVelocityPairs are used." )

        elseif haskey(train,:F_T_pairs_kmh) && train[:F_T_pairs_kmh]!=nothing
            println("WARNING at checking the input for the train: There are values for tractiveEffortVelocityPairs and F_T_pairs_kmh. The values for tractiveEffortVelocityPairs are used." )
        end

    elseif haskey(train,:F_T_pairs) && train[:F_T_pairs]!=nothing
        pairs = train[:F_T_pairs]
        velocityMultiplier = 1.0

        if haskey(train,:F_T_pairs_kmh) && train[:F_T_pairs_kmh]!=nothing
            println("WARNING at checking the input for the train: There are values for F_T_pairs and F_T_pairs_kmh. The values for F_T_pairs are used." )
        end

    elseif haskey(train,:F_T_pairs_kmh) && train[:F_T_pairs_kmh]!=nothing
        velocityMultiplier = 1000/3600
        pairs=[]
        for row in 1:length(train[:F_T_pairs_kmh])
            push!(pairs, [train[:F_T_pairs_kmh][row][1]*velocityMultiplier, train[:F_T_pairs_kmh][row][2]])
        end # for

    else
        error("ERROR at checking the input for the train: There has to be the key tractiveEffortVelocityPairs filled with a list of pairs of velocity and tractive effort.")
    end # if

    # check if the elements of the array have the correct type
    errorDetected=false

    for row in 1:length(pairs)
        if typeof(pairs[row][1]) <: Real && pairs[row][1]>=0.0
        else
            errorDetected=true
            println("ERROR at checking the input for the train: The speed value of train[:tractiveEffortVelocityPairs] in row ", row ," is no real floating point number >=0.0.")
        end
        if typeof(pairs[row][2]) <: Real && pairs[row][2]>=0.0
        else
            errorDetected=true
            println("ERROR at checking the input for the train: The tractive effort value of train[:tractiveEffortVelocityPairs] in row ", row ," is no real floating point number >=0.0.")
        end

        if row>=2 && pairs[row][1] <= pairs[row-1][1]
            errorDetected=true
            println("ERROR at checking the input for the train: The speed value of train[:tractiveEffortVelocityPairs] in row ", row ," (v=",pairs[row][1]," m/s) is not higher than the speed value in the previous row (v=",pairs[row-1][1]," m/s).")
        end
    end # for
    if errorDetected
        error("ERROR at checking the input for the train: Only real floating point number >=0.0 are allowed for speed and tractive effort. The speed values have to be listed from low to high.")
    end

    # create tractiveEffortVelocityPairs
    if pairs[1][1]>0.0    # if there is no F_T for v=0.0, the first known value is used
        newPairs=[]
        push!(newPairs, [0.0, pairs[1][2]])
        println("INFO at checking the input for the train: The tractive effort for v=0.0 m/s is missing. Therefore the first given value F_T(v=",pairs[1][1]," m/s)=",pairs[1][2]," N will be used." )
        for row in 1:length(pairs)
            push!(newPairs, [pairs[row][1], pairs[row][2]])
        end # for
        merge!(train, Dict(:tractiveEffortVelocityPairs => newPairs))
    else
        merge!(train, Dict(:tractiveEffortVelocityPairs => pairs))
    end

    if length(pairs[1])>2
        println("INFO according the train dictionary: Only the first two columns of train[:tractiveEffortVelocityPairs] are used in this tool.")
    end

    return train
end #function checkAndSetTractiveEffortVelocityPairs!

function getDefault_Δv_w(type::String)     # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)
    if type == "passenger"
    # TODO if different passenger or freight trains are posiible, use: if startswith(type, "passenger"). exanples: passengerLocomotivehauled and passengerMotorCoachTrain
        Δv_w=15.0/3.6
    elseif type == "freight"
        Δv_w=0.0
    end # if

    return Δv_w
end #function getDefault_Δv_w!

# function checkAndSetSections!(path::Path)
#     # check the section information
#     if haskey(path,:sections) && path.sections!=nothing
#         # TODO: check typeof(path.sections) == Dict
#         if (haskey(path, :sectionStarts) && path[:sectionStarts]!=nothing) && (haskey(path,:sectionStarts_kmh) && path[:sectionStarts_kmh]!=nothing)
#             println("WARNING at checking the input for the path: There are values for sections, sectionStarts and sectionStarts_kmh. The dictionary sections is used." )

#         elseif haskey(path,:sectionStarts) && path[:sectionStarts]!=nothing
#             println("WARNING at checking the input for the path: There are values for sections and sectionStarts. The dictionary sections is used." )

#         elseif haskey(path,:sectionStarts_kmh) && path[:sectionStarts_kmh]!=nothing
#             println("WARNING at checking the input for the path: There are values for sections and sectionStarts_kmh. The dictionary sections is used." )
#         end
#     elseif haskey(path,:sectionStarts) && path[:sectionStarts]!=nothing
#         # TODO: check typeof(path.sections) == Array
#         createSections!(path, :sectionStarts)

#         if haskey(path,:sectionStarts_kmh) && path[:sectionStarts_kmh]!=nothing
#             println("WARNING at checking the input for the path: There are values for sectionStarts and sectionStarts_kmh. The array sectionStarts is used." )
#         end
#     elseif haskey(path,:sectionStarts_kmh) && path[:sectionStarts_kmh]!=nothing
#         # TODO: check typeof(path.sections) == Array
#         createSections!(path, :sectionStarts_kmh)
#     else
#         error("ERROR at checking the input for the path: The Symbol :sections is missing. It has to be added with a list of sections. Each has to be a dictionary with the keys :s_tart, :s_end, :v_limit and :f_Rp.")
#         section = Dict(:s_start => 0.0,
#                         :s_end => 15.0,
#                         :v_limit => 1000.0/3.6,
#                         :f_Rp => 0.0)
#         merge!(path, Dict(:sections => [section]))
#         return path
#     end

#     sections = path.sections

#     checkedSections = []
#     increasing = false
#     decreasing = false

#     #TODO: throw error for each issue or collect the issues and use the Bool errorDetected like in createSections?

#     # check values for section==1
#     checkAndSetRealNumber!(sections[1], "path.sections[1]", :s_start, "m", 0.0)                      # first point of the section (in m)
#     checkAndSetRealNumber!(sections[1], "path.sections[1]", :s_end, "m", 0.0)                        # first point of the next section (in m)
#     checkAndSetPositiveNumber!(sections[1], "path.sections[1]", :v_limit, "m/s", 1000.0/3.6)    # paths speed limt (in m/s)
#     checkAndSetRealNumber!(sections[1], "path.sections[1]", :f_Rp, "‰", 0.0)                # specific path resistance of the section (in ‰)

#     push!(checkedSections, sections[1])

#     if sections[1][:s_start] < sections[1][:s_end]
#         increasing = true
#     elseif sections[1][:s_start] > sections[1][:s_end]
#         decreasing = true
#     else
#         pop!(checkedSections)
#         println("WARNING at checking the input for the path: The first section of :sections has the same position for starting and end point. The section will be deleted and not used in the tool.")
#     end


#     for sectionNr in 2:length(sections)
#         checkAndSetRealNumber!(sections[sectionNr], "path.sections["*string(sectionNr)*"]", :s_start, "m", sections[sectionNr-1][:s_end])                      # first point of the section (in m)
#         # TODO how to define default values? which has to be checked and defined fist? s_end-1 and s_start need each other as default values
#         #if sectionNr < length(sections) && haskey(sections[sectionNr], :s_start) && sections[sectionNr][:s_start]!=nothing && typeof(sections[sectionNr][:s_start]) <: Real
#         #    defaultEnd = sections[sectionNr+1][:s_start]
#         #end
#         defaultEnd = sections[sectionNr][:s_start]  # so the default value for s_end creates a sections of lenght=0.0   #TODO should be changed!
#         checkAndSetRealNumber!(sections[sectionNr], "path.sections["*string(sectionNr)*"]", :s_end, "m", defaultEnd)                        # first point of the next section (in m)
#         checkAndSetPositiveNumber!(sections[sectionNr], "path.sections["*string(sectionNr)*"]", :v_limit, "m/s", 1000.0/3.6)    # paths speed limt (in m/s)
#         checkAndSetRealNumber!(sections[sectionNr], "path.sections["*string(sectionNr)*"]", :f_Rp, "‰", 0.0)                # specific path resistance of the section (in ‰)

#         push!(checkedSections, sections[sectionNr])

#         # compare the section's start and end position
#         if sections[sectionNr][:s_start] < sections[sectionNr][:s_end]
#             increasing = true
#         elseif sections[sectionNr][:s_start] > sections[sectionNr][:s_end]
#             decreasing = true
#         else
#             pop!(checkedSections)
#             println("INFO at checking the input for the path: The ",sectionNr,". section of :sections has the same position for starting and end point. The section will be deleted and not used in the tool.")
#         end
#         if increasing && decreasing
#             error("ERROR at checking the input for the path: The positions of the :sections are not increasing/decreasing consistently. The direction in the ",sectionNr,". section differs from the previous.")
#         end


#         if length(checkedSections)>1 && sections[sectionNr][:s_start] != checkedSections[end-1][:s_end]
#             error("ERROR at checking the input for the path.sections: The starting position of the ",section,". section (s=",sections[sectionNr][:s_start]," m) does not euqal the last position of the previous section(s=",checkedSections[end-1][:s_end]," m). The sections have to be sequential.")
#             # TODO: maybe if there is a gab create a new section and only if there a jumps in the wrong direction throw an error?
#         end
#     end #for

#     return path
# end #function checkAndSetSections!

# function createSections!(path::Path, key::Symbol)
#     # read the section starting positions and corresponding information
#     if key == :sectionStarts
#         sectionStartsArray = path[:sectionStarts]
#         conversionFactor = 1.0                                # conversion factor between the units m/s and m/s

#         if haskey(path,:sectionStarts) && path[:sectionStarts_kmh]!=nothing
#             println("WARNING at checking the input for the path: There are values for sectionStarts and sectionStarts_kmh. The values for sectionStarts are used." )
#         end
#     elseif key == :sectionStarts_kmh
#         sectionStartsArray = path[:sectionStarts_kmh]
#         conversionFactor = 1/3.6                              # conversion factor between the units km/h and m/s
#     elseif key == :characteristic_sections
#         sectionStartsArray = path[:characteristic_sections]
#         conversionFactor = 1/3.6                              # conversion factor between the units km/h and m/s
#     else
#         error("ERROR at checking the input for the path: The keyword sectionStarts or sectionStarts_kmh is missing. The sections can not be created without them.")
#     end # if

#     # check if the array is correct and if elements of the array have the correct type and valid values
#     errorDetected = false
#     if length(sectionStartsArray)<2
#         error("ERROR at checking the input for the path: The keyword ",key," needs at least two rows for two points each with the three columns [s, v_limit, f_Rp].")
#     end

#     for row in 1:length(sectionStartsArray)
#         if length(sectionStartsArray[row])>=3
#             if length(sectionStartsArray[row])>3
#                 println("INFO at checking the input for the path: Only the first three columns of sectionStartsArray are used in this tool.")
#             end
#         else
#             error("ERROR at checking the input for the path: The keyword ",key," needs to be filled with the three columns [s, v_limit, f_Rp].")
#         end

#         if !(typeof(sectionStartsArray[row][1]) <: Real)
#             errorDetected=true
#             println("ERROR at checking the input for the path: The position value (column 1) of ",key," in row ", row ," is no real floating point number.")
#         end
#         if !(typeof(sectionStartsArray[row][2]) <: Real && sectionStartsArray[row][2] >= 0.0)
#             errorDetected=true
#             println("ERROR at checking the input for the path: The speed limit (column 2) of ",key," in row ", row ," is no real floating point number >=0.0.")
#         end
#         if !(typeof(sectionStartsArray[row][3]) <: Real)
#             errorDetected=true
#             println("ERROR at checking the input for the path: The tractive effort value (column 3) of ",key," in row ", row ," is no real floating point number.")
#         end
#     end # for
#     if errorDetected
#         error("ERROR at checking the input for the path: The values of ",key," have to be corrected.")
#     end


#     sections = []
#     for row in 2:length(sectionStartsArray)
#         s_start = sectionStartsArray[row-1][1]                    # first point of the section (in m)
#         s_end = sectionStartsArray[row][1]                        # first point of the next section (in m)
#         v_limit = sectionStartsArray[row-1][2]*conversionFactor   # paths speed limt (in m/s)
#         f_Rp = sectionStartsArray[row-1][3]                       # specific path resistance of the section (in ‰)

#         section = Dict(:s_start => s_start,
#                         :s_end => s_end,
#                         :v_limit => v_limit,
#                         :f_Rp => f_Rp)
#         push!(sections, section)
#     end # for
#     # s_start in first entry defines the path's beginning
#     # s_end in last entry defines the path's ending

#     merge!(path, Dict(:sections => sections))
#     return path
# end #function createSections!

# function checkAndSetPOIs!(path::Path)
#     # read the section starting positions and corresponding information
#     if haskey(path, :pointsOfInterest)
#         # if path.poi != nothing
#             pointsOfInterest = path[:points_of_interest]

#             sortingNeeded = false
#             errorDetected = false
#             for element in 1:length(pointsOfInterest)
#                 if typeof(pointsOfInterest[element]) <: Real
#                     if element > 1
#                         if pointsOfInterest[element] < pointsOfInterest[element-1]
#                             sortingNeeded = true
#                             println("INFO at checking the input for the path: The point of interest in element ", element ," (",pointsOfInterest[element]," m) has to be higher than the value of the previous element (",pointsOfInterest[element-1]," m). The points of interest will be sorted.")
#                         end
#                     end
#                 else
#                     errorDetected = true
#                     println("ERROR at checking the input for the path: The point of interest in element ", element ," is no real floating point number.")
#                 end
#             end # for

#             if errorDetected
#                 error("ERROR at checking the input for the path: The values of pointsOfInterest have to be corrected.")
#             end
#             if sortingNeeded == true
#                 sort!(pointsOfInterest)
#             end

#             copiedPOIs = []
#             for element in 1:length(pointsOfInterest)
#                 if element == 1
#                     push!(copiedPOIs, pointsOfInterest[element])
#                 elseif element > 1 && pointsOfInterest[element] > pointsOfInterest[element-1]
#                     push!(copiedPOIs, pointsOfInterest[element])
#                 end
#             end # for
#             path[:points_of_interest ] = copiedPOIs

#         # else
#         #     println("INFO at checking the input for the path: The key pointsOfInterest exists but without values.")
#         #     delete!(path, :points_of_interest)
#         # end
#     end

#     return path
# end #function checkAndSetPOIs!

#function informAboutUnusedKeys(dictionary::Dict, dictionaryType::String)         # inform the user which Symbols of the input dictionary are not used in this tool
function informAboutUnusedKeys(allKeys::AbstractVector, usedKeys::Vector{Symbol}, dictionaryType::String)         # inform the user which Symbols of the input dictionary are not used in this tool
    unusedKeys = []
    # find unused keys in allKeys
    for key in allKeys
        used = false
        for usedKey in usedKeys
            if key == usedKey
                used = true
                break
            end
        end
        if !used
            push!(unusedKeys, key)
        end
    end

    if length(unusedKeys)>0
        println("INFO at checking the input for the ",dictionaryType,": The following Keywords are not used in this tool:")
        for key in unusedKeys
            println("  - ",key)
        end
    end
end #function informAboutUnusedKeys
