#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Martin Scheidt"
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

    Settings(massModel, stepVariable, stepSize, approxLevel, outputDetail, outputFormat, outputDir)

end #function Settings() # constructor


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
        if data["schema_version"] != "2022.05"
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

    Path(name, id, uuid, poi, sections)

end #function Path() # constructor

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
