#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Martin Scheidt, Max Kannenberg"
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
Settings(mass_point, :distance, 20, 3, running_time, :dataframe)
```
"""
function Settings(file="DEFAULT")

    ## default values
    massModel    = :mass_point
    stepVariable = :distance
    stepSize     = 20
    approxLevel  = 3
    outputDetail = :running_time
    outputFormat = :dataframe

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
                    "enum": [ "dataframe", "dict" ]
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

        ## set the variables in "settings"
        haskey(settings, "massModel")    ? massModel    = Symbol(settings["massModel"])    : nothing
        haskey(settings, "stepVariable") ? stepVariable = Symbol(settings["stepVariable"]) : nothing
        haskey(settings, "stepSize")     ? stepSize     =        settings["stepSize"]      : nothing
        haskey(settings, "approxLevel")  ? approxLevel  =        settings["approxLevel"]   : nothing
        haskey(settings, "outputDetail") ? outputDetail = Symbol(settings["outputDetail"]) : nothing
        haskey(settings, "outputFormat") ? outputFormat = Symbol(settings["outputFormat"]) : nothing
    end

    Settings(massModel, stepVariable, stepSize, approxLevel, outputDetail, outputFormat)

end #function Settings() # outer constructor

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

        ## set the variables in "path"
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

end #function Path() # outer constructor

"""
    Train(file, type = :YAML)

Train is a datastruture for calculation context.
The function Train() will create a train to use in calculations.
Supported formats for the YAML files are: railtoolkit/schema (2022.05)

# Example
```jldoctest
julia> my_train = Train("file.yaml") # will generate a train from a YAML file.
Train(variables)
```
"""
function Train(file, type = :YAML)

    ## default values
    name          = ""            #
    id            = ""            #
    uuid          = UUIDs.uuid4() #
    length        = 0             # in meter
    m_train_full  = 0             # in kilogram
    m_train_empty = 0             # in kilogram
    m_loco        = 0             # in kilogram
    m_td          = 0             # in kilogram
    m_tc          = 0             # in kilogram
    m_car_full    = 0             # in kilogram
    m_car_empty   = 0             # in kilogram
    ξ_train       = 1.08          # rotation mass factor, source: "Fahrdynamik des Schienenverkehrs" by Wende, 2003, p. 13 for "Zug, überschlägliche Berechnung"
    ξ_loco        = 1.09          # rotation mass factor
    ξ_cars        = 1.06          # rotation mass factor
    transportType = :freight      # "freight" or "passenger" for resistance calculation
    v_limit       = 140           # in m/s (default 504 km/h)
    a_braking     = 0             # in m/s^2, todo: implement as function
    f_Rtd0        = 0             # coefficient for basic resistance due to the traction units driving axles (in ‰)
    f_Rtc0        = 0             # coefficient for basic resistance due to the traction units carring axles (in ‰)
    F_Rt2         = 3000          # coefficient for air resistance of the traction units (in N)
    f_Rw0         = 0             # coefficient for the consists basic resistance (in ‰)
    f_Rw1         = 0             # coefficient for the consists resistance to rolling (in ‰)
    f_Rw2         = 0             # coefficient fo the consistsr air resistance (in ‰)
    F_v_pairs     = []            # [v in m/s, F_T in N]

    ## load from file
    if type == :YAML

        data = YAML.load(open(file))
        if data["schema"] != "https://railtoolkit.org/schema/rolling-stock.json"
            error("Could not load path file '$file'.\n
                    YAML format is not recognized.
                    Currently supported: railtoolkit/schema/rolling-stock (2022.05)")
        end
        if data["schema_version"] != "2022.05"
            error("Could not load path file '$file'.\n
                    YAML format is not recognized.
                    Currently supported: railtoolkit/schema/rolling-stock (2022.05)")
        end

        ## JSON schema for YAML-file validation
        railtoolkit_schema = Schema("""{
            "required": [ "schema", "schema_version" ],
            "anyOf": [
            {"required": [ "trains" ] },
            {"required": [ "vehicles" ] }
            ],
            "properties": {
            "schema": {
                "description": "Identifier of the schema",
                "enum": [ "https://railtoolkit.org/schema/rolling-stock.json" ]
            },
            "schema_version": {
                "description": "Version of the schema",
                "type": "string",
                "pattern": "[2-9][0-9][0-9][0-9].[0-1][0-9]"
            },
            "trains": {
                "type": "array",
                "minItems": 1,
                "items": {
                "required": [ "name", "id", "formation" ],
                "type": "object",
                "properties": {
                    "id": {
                    "description": "Identifier of the train",
                    "type": "string"
                    },
                    "name": {
                    "description": "Name of the train",
                    "type": "string"
                    },
                    "UUID": {
                    "description": "The unique identifier for a train",
                    "type": "string",
                    "format": "uuid"
                    },
                    "formation": {
                    "description": "Collection of vehicles that form the train",
                    "type": "array",
                    "minItems": 1,
                    "uniqueItems": false,
                    "items": {
                        "type": "string"
                    }
                    }
                }
                }
            },
            "vehicles": {
                "type": "array",
                "minItems": 1,
                "items": {
                "required": [ "name", "id", "vehicle_type", "length", "mass" ],
                "type": "object",
                "properties": {
                    "air_resistance": {
                    "description": "coefficient for air resistance in permil",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "base_resistance": {
                    "description": "coefficient for basic resistance in permil",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "id": {
                    "description": "Identifier of the vehicle",
                    "type": "string"
                    },
                    "length": {
                    "description": "The length of the vehicle in meter",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "load_limit": {
                    "description": "The maximum permitted load of the vehicle in metric ton",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "mass_traction": {
                    "description": "The mass on the powered axles of the vehicle in metric ton",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "mass": {
                    "description": "The empty mass of the vehicle in metric ton",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "name": {
                    "description": "Name of the vehicle",
                    "type": "string"
                    },
                    "picture": {
                    "description": "A URI with a picture for humans",
                    "type": "string",
                    "format": "uri"
                    },
                    "power_type": {
                    "description": "Type of propulsion",
                    "enum": [ "diesel", "electric", "steam" ]
                    },
                    "rolling_resistance": {
                    "description": "coefficient for resistance of rolling axles in permil",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "rotation_mass": {
                    "description": "Factor for rotating mass; >= 1",
                    "type": "number",
                    "minimum": 1
                    },
                    "speed_limit": {
                    "description": "Maximum permitted speed in kilometers per hour",
                    "type": "number",
                    "exclusiveMinimum": 0
                    },
                    "tractive_effort": {
                    "description": "Tractive effort as pairs of speed in kilometers per hour and tractive force in newton",
                    "type": "array",
                    "minItems": 3,
                    "uniqueItems": true,
                    "items": {
                        "type": "array",
                        "minItems": 2,
                        "maxItems": 2,
                        "uniqueItems": true,
                        "items": {
                        "type": "number",
                        "minimum": 0
                        }
                    }
                    },
                    "UUID": {
                    "description": "The unique identifier for a vehicle",
                    "type": "string",
                    "format": "uuid"
                    },
                    "vehicle_type": {
                    "description": "Type of vehicle",
                    "enum": [ "traction unit", "freight", "passenger", "multiple unit" ]
                    }
                }
                }
            }
            }
        }""")

        try
            validate(railtoolkit_schema, data)
        catch err
            error("Could not load path file '$file'.\n
                    YAML format is not recognized.
                    Currently supported: railtoolkit/schema/rolling-stock (2022.05)")
        end

    else
        error("Unknown file type '$type'")
    end #if type

    trains = data["trains"]
    Base.length(trains) > 1 ? println("WARNING: the loaded file contains more than one train. Using only the first!") : nothing
    Base.length(trains) == 0 ? error("No train present in file '$file'") : nothing
    train = trains[1]
    used_vehicles = unique(train["formation"])

    included_vehicles = []
    for vehicle in data["vehicles"]
        push!(included_vehicles,vehicle["id"])
    end

    ## test if all vehicles of the formation are avilable
    for vehicle in used_vehicles
        vehicle ∉ included_vehicles ? error("'$vehicle' is not present in '$file'") : nothing
    end

    ## gather the count of vehicles and usage in the formation
    vehicles = NamedTuple[]
    for vehicle in data["vehicles"]
        if vehicle["id"] in used_vehicles
            n = count(==(vehicle["id"]),train["formation"])
            type = vehicle["vehicle_type"]
            type == "traction unit" || type == "multiple unit" ? propulsion    = true       : propulsion = false
            type == "passenger"     || type == "multiple unit" ? transportType = :passenger : nothing
            push!(vehicles, (data=vehicle, n=n, propulsion=propulsion) )
        end
    end

    ## set the variables in "train"
    name = train["name"]
    id   = train["id"]
    haskey(train, "UUID") ? uuid = parse(UUID, train["UUID"] ) : nothing
    transportType == :freight ? a_braking = -0.225 : a_braking = -0.375  # set a default a_braking value depending on the train type

    ## set the variables for all vehicles
    for vehicle in vehicles
        length           += vehicle.data["length"] * vehicle.n
        m_train_full     += vehicle.data["mass"]   * vehicle.n * 1000 # in kg
        m_train_empty    += vehicle.data["mass"]   * vehicle.n * 1000 # in kg
        haskey(vehicle.data, "load_limit")    ?
            m_train_full += vehicle.data["load_limit"] * vehicle.n * 1000 :  # in kg
            nothing
        haskey(vehicle.data, "speed_limit")   ?
            v_limit > vehicle.data["speed_limit"]/3.6 ? v_limit = vehicle.data["speed_limit"]/3.6 : nothing :
            nothing
    end

    ## divide vehicles in propulsion and non-propulsion
    loco = []
    for i in 1:Base.length(vehicles)
        if vehicles[i].propulsion
            push!(loco, vehicles[i])
            deleteat!(vehicles, i)
        end
    end
    Base.length(loco)  > 1 ? println("WARNING: the loaded file contains more than one traction unit or multiple unit. Using only the first!") : nothing
    loco[1].n     > 1 ? println("WARNING: the loaded file contains more than one traction unit or multiple unit. Using only one!") : nothing
    Base.length(loco) == 0 ? error("No traction unit or multiple unit present in file '$file'") : nothing
    loco = loco[1].data
    cars = vehicles

    ## set the variables for locos
    m_loco= loco["mass"] * 1000
    haskey(loco, "a_braking")          ? a_braking = loco["a_braking"]                : nothing
    haskey(loco, "base_resistance")    ? f_Rtd0 = loco["base_resistance"]             : nothing
    haskey(loco, "rolling_resistance") ? f_Rtc0 = loco["rolling_resistance"]          : nothing
    haskey(loco, "air_resistance")     ? F_Rt2  = loco["air_resistance"] * g * m_loco : nothing
    haskey(loco, "mass_traction")      ? m_td   = loco["mass_traction"] * 1000        : m_td = m_t
    haskey(loco, "rotation_mass")      ? ξ_loco = loco["rotation_mass"]               : nothing
    m_tc = m_loco- m_td
    haskey(loco, "tractive_effort")    ? F_v_pairs = loco["tractive_effort"] : F_v_pairs = [ [0.0, m_td * g * μ],[v_limit*3.6, m_td * g * μ] ]
    F_v_pairs = reduce(hcat,F_v_pairs)'       # convert to matrix
    F_v_pairs[:,1] ./= 3.6                    # convert km/h to m/s
    F_v_pairs = tuple.(eachcol(F_v_pairs)...) # convert each row to tuples

    ## set the variables for cars
    if !isempty(cars)
        resis_base = []
        resis_roll = []
        resis_air  = []
        rotMassFac = []
        for car in cars
            haskey(car.data, "base_resistance")    ?
                append!(resis_base,repeat([car.data["base_resistance"]],car.n))    :
                append!(resis_base,repeat([f_Rw0],car.n))
            haskey(car.data, "rolling_resistance") ?
                append!(resis_roll,repeat([car.data["rolling_resistance"]],car.n)) :
                append!(resis_roll,repeat([f_Rw1],car.n))
            haskey(car.data, "air_resistance")     ?
                append!(resis_air,repeat([car.data["air_resistance"]],car.n))      :
                append!(resis_air, repeat([f_Rw2],car.n))
            haskey(car.data, "rotation_mass") ?
                append!(rotMassFac,repeat([(car.data["rotation_mass"],car.data["mass"])],car.n)) :
                append!(rotMassFac,repeat([(ξ_cars ,car.data["mass"])],car.n))
            m_car_empty += car.data["mass"] * car.n * 1000 # in kg
            m_car_full  += car.data["mass"] * car.n * 1000 # in kg
            haskey(car.data, "load_limit")    ?
                m_car_full += car.data["load_limit"] * car.n * 1000 :  # in kg
                nothing
        end
        f_Rw0   = Statistics.mean(resis_base)
        f_Rw1   = Statistics.mean(resis_roll)
        f_Rw2   = Statistics.mean(resis_air)
        carRotMass = 0
        for elem in rotMassFac
            carRotMass += elem[1]*elem[2] * 1000 # in kg
        end
        ξ_cars  = carRotMass/m_car_empty
        ξ_train = (ξ_loco * m_loco+ carRotMass)/m_train_empty
    else
        ξ_cars  = 0
        ξ_train = ξ_loco
    end

    Train(
        name, id, uuid, length,
        m_train_full, m_td, m_tc, m_car_full,
        ξ_train, ξ_loco, ξ_cars,
        transportType, v_limit,
        a_braking,
        f_Rtd0, f_Rtc0, F_Rt2, f_Rw0, f_Rw1, f_Rw2,
        F_v_pairs
    )

end #function Train() # outer constructor

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
        :type => type,                 # type of behavior section: "breakFree", "clearing", "accelerating", "cruising", "downhillBraking", "diminishing", "coasting", "braking" or "halt"
        :length => 0.0,                # total length  (in m)
        :s_entry => s_entry,           # first position (in m)
        :s_exit => 0.0,                # last position  (in m)
        :t => 0.0,                     # total running time (in s)
        :v_entry => v_entry,           # entry speed (in m/s)
        :v_exit => 0.0,                # exit speed (in m/s)
        :dataPoints => [startingPoint] # list of identifiers of the containing data points starting with the initial point
    )
    return BS
end #function createBehaviorSection
