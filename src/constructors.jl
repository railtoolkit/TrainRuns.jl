#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Martin Scheidt, Max Kannenberg"
# __copyright__     = "2022"
# __license__       = "ISC"

"""
    load([file::String])

Loads data from a `file`. A schema must be specified in the file.
The content of the `file` is then checked using the specified schema.

Returns a Dict() for further processing by the type constructors.

# Supported file formats and schemas
## File formats
* YAML
* JSON

## Schemas
* [railtoolkit-schema](https://github.com/railtoolkit/schema)
    * Version v2022.05

# Example
```julia-repl
julia> load("data/variables.yaml")
Dict{Any, Any} with data_variables
```
"""
function load(file::String)::Dict
    @debug "loading file - passed file: $file"
    file_extension = lowercase(split(file, ".")[end])
    @debug "loading file - detected file extension: $file_extension"
    if file_extension == "yml" || file_extension == "yaml"
        data = YAML.load(open(file))
    elseif file_extension == "json"
        data = JSON.parsefile(file)
        #else #new file formats here
        #
    else
        @info "The file format with the extension '$file_extension' is not supported."
        @info "Supported file formats are: YAML and JSON."
        @info "For a schema format see: https://github.com/railtoolkit/schema"
        error("Can not load '$file'. Unsupported file extension!")
    end

    if !haskey(data, "schema")
        error("Can not load '$file'. No attribute with 'schema' found!")
    end
    if !haskey(data, "schema_version")
        error("Can not load '$file'. No attribute with 'schema_version' found!")
    end

    schema = get_schema(data)
    if !JSONSchema.isvalid(schema, data)
        @info "Could not parse file '$file'. Not a valide schema format."
        @info "Currently supported schemas: railtoolkit/schema v2022.05"
        @info "For the schema format see: https://github.com/railtoolkit/schema"
        error("Can not load '$file'. Format not recognized!")
    end

    return data
end

"""
    get_schema([data::Dict])

Extracts the schema from a data dictionary.
A schema and a schema version must be specified in the data dictionary.
The schema name and a schema version are combined to a string which is dependend on Artifact.toml entry.

Returns a JSONSchema.Schema type.

# Example
```julia-repl
julia> get_schema(data)
A JSONSchema
```
"""
function get_schema(data::Dict)::Schema
    ## assumption - schema come in the form of a URI: (http://)schema_name/schema_subtype

    schema = parse(URI, data["schema"])
    @debug "loading file - detected schema: $schema"
    schema_version = lowercase(data["schema_version"])
    @debug "loading file - detected schema version: $schema_version"

    fragment = URIs.splitpath(schema)
    schema_subtype = pop!(fragment)
    @debug "loading file - detected schema subtype: $schema_subtype"

    schema_name = join([schema.host, schema_version], "-")
    schema_name = replace(schema_name, r"\." => "-")
    @debug "loading file - artifact string: $schema"

    if schema_name == "railtoolkit-org-2022-05"
        artifact_path = artifact"railtoolkit-org-2022-05" # definied by Artifacts.toml
        schema_path = joinpath(artifact_path, "schema-2022.05", "src", schema_subtype) # depending on the loaded artifact
    else
        @info "The schema string '$schema_name' is not in the list of supported artifacts."
        @info "Currently supported schemas: railtoolkit/schema v2022.05"
        @info "For the schema format see: https://github.com/railtoolkit/schema"
        error("The provided schema '$schema' version '$schema_version' is not recognized!")
    end
    @debug "loading file - schema path: $schema_path"

    return Schema(JSON.parsefile(schema_path))
end

"""
    Settings([file::String]; <keyword arguments>)

Create a settings object for [`trainrun`](@ref).

`file` may be used to load settings in YAML format.

# Arguments
- `massModel::Symbol` model of train mass. `:mass_point`_(default)_ or `:homogeneous_strip`
- `stepVariable::Symbol`: step method `:distance`_(default)_, `:time` or `:velocity`
- `stepSize::Number=20`: unit depends on `stepVariable` - (_meter_, _seconds_ or _meter/second_)
- `approxLevel::Number=3`: used when rounding or iterating.
- `outputDetail:Symbol`: `:running_time`_(default)_, `:points_of_interest`, `:data_points` or `:driving_course`
- `outputFormat::Symbol`: `:dataframe`_(default)_ or `:vector`
- `verbosity::Symbol`: `:unset`_(default)_, `:trace`, `:debug`, `:info`, `:warn`, `:error`, or `:fatal`

# Example
```julia-repl
julia> my_settings = Settings()
Settings(:mass_point, :distance, 20, 3, :running_time, :dataframe)
```
"""
function Settings(
        file = "DEFAULT";
        massModel::Symbol = :mass_point,
        stepVariable::Symbol = :distance,
        stepSize::Number = 20,
        approxLevel::Number = 3,
        outputDetail::Symbol = :running_time,
        outputFormat::Symbol = :dataframe,
        verbosity::Symbol = :unset
)
    ## load from file
    if file != "DEFAULT"

        ## JSON schema for YAML-file validation
        schema = Schema(
            """{
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
            "enum": [ "running_time", "points_of_interest", "data_points", "driving_course" ]
        },
        "outputFormat": {
            "description": "Output format",
            "type": "string",
            "enum": [ "dataframe", "vector" ]
        },
        "verbosity": {
            "description": "Output format",
            "type": "string",
            "enum": [ "unset", "trace", "debug", "info", "warn", "error", "fatal" ]
        }
    }
}""",
        )

        settings = YAML.load(open(file))["settings"]

        ## validate the loaded file
        if !isvalid(schema, settings)
            @warn "Could not load settings file '$file'. Format is not recognized!" "Using default as fall back."
            settings = Dict()
        end

        ## set the variables in "settings"
        haskey(settings, "massModel") ? massModel = Symbol(settings["massModel"]) : nothing
        haskey(settings, "stepVariable") ? stepVariable = Symbol(settings["stepVariable"]) :
        nothing
        haskey(settings, "stepSize") ? stepSize = settings["stepSize"] : nothing
        haskey(settings, "approxLevel") ? approxLevel = settings["approxLevel"] : nothing
        haskey(settings, "outputDetail") ? outputDetail = Symbol(settings["outputDetail"]) :
        nothing
        haskey(settings, "outputFormat") ? outputFormat = Symbol(settings["outputFormat"]) :
        nothing
        haskey(settings, "verbosity") ? verbosity = Symbol(settings["verbosity"]) : nothing
    end

    Settings(
        massModel,
        stepVariable,
        stepSize,
        approxLevel,
        outputDetail,
        outputFormat,
        verbosity
    )
end #function Settings() # outer constructor

"""
    Path(file, type=:YAML)

Create a running path object for [`trainrun`](@ref).

Supported formats are: [railtoolkit/schema (2022.05)](https://doi.org/10.5281/zenodo.6522824).
As of now only `:YAML` is supported as filetype.

# Example
```julia-repl
julia> my_path = Path("file.yaml")
Path(variables)
```
"""
function Path(file::String)

    ## default values
    name = ""
    id = ""
    uuid = UUIDs.uuid4()
    poi = []
    sections = []

    ## process flags
    POI_PRESENT = false

    data = load(file)
    if !haskey(data, "paths")
        error("Can not load '$file'. No collection with 'paths' found!")
    end

    length(data["paths"]) > 1 ?
    (@warn "The loaded file contains more than one path. Using only the first!") :
    nothing
    path = data["paths"][1]

    ## set the variables in "path"
    # required
    name = path["name"]
    id = path["id"]
    tmp_sec = path["characteristic_sections"]
    # optional
    haskey(path, "UUID") ? uuid = parse(UUID, path["UUID"]) : nothing
    haskey(path, "points_of_interest") ? POI_PRESENT = true : nothing
    haskey(path, "points_of_interest") ? tmp_points = path["points_of_interest"] :
    nothing

    ## process characteristic sections
    sort!(tmp_sec, by = x -> x[1])
    for row in 2:length(tmp_sec)
        s_start = tmp_sec[row - 1][1]     # first point of the section (in m)
        s_end = tmp_sec[row][1]       # first point of the next section (in m)
        v_limit = tmp_sec[row - 1][2] / 3.6 # paths speed limt (in m/s)
        f_Rp = tmp_sec[row - 1][3]     # specific path resistance of the section (in ‰)

        section = Dict(
            :s_start => s_start, :s_end => s_end, :v_limit => v_limit, :f_Rp => f_Rp)
        push!(sections, section)
    end #for row
    # s_start in first entry defines the path's beginning
    # s_end in last entry defines the path's ending

    ## process points of interest
    if POI_PRESENT
        sort!(tmp_points, by = x -> x[1])
        for elem in tmp_points
            station = elem[1]     # station in m
            label = elem[2]     # name
            measure = elem[3]     # front or rear

            point = Dict(:station => station, :label => label, :measure => measure)
            push!(poi, point)
        end #for elem
    end #if !isempty(points)

    Path(name, id, uuid, poi, sections)
end #function Path() # outer constructor

"""
    Path(characteristic_sections::DataFrame[, points_of_interest::DataFrame]; <keyword arguments>)

Create a running path object for [`trainrun`](@ref).

# Arguments
- `name::String`: name or description
- `id::String`: short string as identifier
- `uuid::UUID`: unique identifier


`characteristic_sections` DataFrame needs the columns:
- :position
- :speed
- :resistance


`points_of_interest` DataFrame needs the columns:
- :position
- :label
- :measure

# Example
```julia-repl
julia> my_path = Path(characteristic_sections)
Path(variables)
```
"""
function Path(
        characteristic_sections::DataFrame,
        points_of_interest::DataFrame = DataFrame(
            position = Real[],
            label = String[],
            measure = String[]
        );
        name::String = "",
        id::String = "",
        uuid::UUID = UUIDs.uuid4()
)

    ## points_of_interest
    poi = []
    for elem in eachrow(select(points_of_interest, [:position, :label, :measure]))
        station = elem[:position] # station in m
        label = elem[:label]    # name
        measure = elem[:measure]  # front or rear
        #
        point = Dict(:station => station, :label => label, :measure => measure)
        push!(poi, point)
    end #for elem in poi

    ## characteristic_sections
    cs = select(characteristic_sections, [:position, :speed, :resistance])
    # create s_end
    sort!(cs, [:position])
    next_position = circshift(cs[!, :position], -1)
    next_position = convert(Vector{Union{Missing, Real}}, next_position)
    next_position[end] = missing
    allowmissing!(cs)
    cs.next_position = next_position
    #
    sections = []
    for elem in eachrow(dropmissing(cs))
        s_start = elem[:position]      # first point of the section (in m)
        s_end = elem[:next_position] # first point of the next section (in m)
        v_limit = elem[:speed]         # paths speed limt (in m/s)
        f_Rp = elem[:resistance]    # specific path resistance of the section (in ‰)
        #
        section = Dict(
            :s_start => s_start, :s_end => s_end, :v_limit => v_limit, :f_Rp => f_Rp)
        push!(sections, section)
    end #for elem in cs

    return Path(name, id, uuid, poi, sections)
end #function Path() # outer constructor

"""
    Train(file, type=:YAML)

Create a train object for [`trainrun`](@ref).

Supported formats are: [railtoolkit/schema (2022.05)](https://doi.org/10.5281/zenodo.6522824).
As of now only `:YAML` is supported as filetype.

# Example
```julia-repl
julia> my_train = Train("file.yaml")
Train(variables)
```
"""
function Train(file::String)

    ## default values
    name = ""            #
    id = ""            #
    uuid = UUIDs.uuid4() #
    length = 0             # in meter
    m_train_full = 0             # in kilogram
    m_train_empty = 0             # in kilogram
    m_loco = 0             # in kilogram
    m_td = 0             # in kilogram
    m_tc = 0             # in kilogram
    m_car_full = 0             # in kilogram
    m_car_empty = 0             # in kilogram
    ξ_train = 1.08          # rotation mass factor, source: "Fahrdynamik des Schienenverkehrs" by Wende, 2003, p. 13 for "Zug, überschlägliche Berechnung"
    ξ_loco = 1.09          # rotation mass factor
    ξ_cars = 1.06          # rotation mass factor
    transportType = :freight      # "freight" or "passenger" for resistance calculation
    v_limit = 140           # in m/s (default 504 km/h)
    a_braking = 0             # in m/s^2, TODO: implement as function
    f_Rtd0 = 0             # coefficient for basic resistance due to the traction unit's driving axles (in ‰)
    f_Rtc0 = 0             # coefficient for basic resistance due to the traction unit's carring axles (in ‰)
    f_Rt2 = 0             # coefficient for air resistance of the traction unit (in ‰)
    f_Rw0 = 0             # coefficient for the consist's basic resistance (in ‰)
    f_Rw1 = 0             # coefficient for the consist's resistance to rolling (in ‰)
    f_Rw2 = 0             # coefficient for the consist's air resistance (in ‰)
    F_v_pairs = []            # [v in m/s, F_T in N]

    ## validation
    data = load(file)
    if !haskey(data, "trains")
        error("Can not load '$file'. No collection with 'trains' found!")
    end
    if !haskey(data, "vehicles")
        error("Can not load '$file'. No collection with 'vehicles' found!")
    end

    trains = data["trains"]
    Base.length(trains) > 1 ?
    (@warn "The loaded file contains more than one train. Using only the first!") : nothing
    Base.length(trains) == 0 ? throw(DomainError("No train present in file '$file'")) :
    nothing
    train = trains[1]
    used_vehicles = unique(train["formation"])

    included_vehicles = []
    for vehicle in data["vehicles"]
        push!(included_vehicles, vehicle["id"])
    end

    ## test if all vehicles of the formation are avilable
    for vehicle in used_vehicles
        vehicle ∉ included_vehicles ?
        throw(DomainError("'$vehicle' is not present in '$file'")) : nothing
    end

    ## gather the count of vehicles and usage in the formation
    vehicles = NamedTuple[]
    for vehicle in data["vehicles"]
        if vehicle["id"] in used_vehicles
            n = count(==(vehicle["id"]), train["formation"])
            type = vehicle["vehicle_type"]
            type == "traction unit" || type == "multiple unit" ? propulsion = true :
            propulsion = false
            type == "passenger" || type == "multiple unit" ? transportType = :passenger :
            nothing
            push!(vehicles, (data = vehicle, n = n, propulsion = propulsion))
        end
    end

    ## set the variables in "train"
    name = train["name"]
    id = train["id"]
    haskey(train, "UUID") ? uuid = parse(UUID, train["UUID"]) : nothing
    transportType == :freight ? a_braking = -0.225 : a_braking = -0.375  # set a default a_braking value depending on the train type
    #TODO: add source: Brünger, Dahlhaus, 2014 p. 74 (see formulary.jl)

    ## set the variables for all vehicles
    for vehicle in vehicles
        length += vehicle.data["length"] * vehicle.n
        m_train_full += vehicle.data["mass"] * vehicle.n * 1000 # in kg
        m_train_empty += vehicle.data["mass"] * vehicle.n * 1000 # in kg
        haskey(vehicle.data, "load_limit") ?
        m_train_full += vehicle.data["load_limit"] * vehicle.n * 1000 :  # in kg
        nothing
        haskey(vehicle.data, "speed_limit") ?
        v_limit > vehicle.data["speed_limit"] / 3.6 ?
        v_limit = vehicle.data["speed_limit"] / 3.6 : nothing : nothing
    end

    ## divide vehicles in propulsion and non-propulsion
    loco = []
    for i in 1:Base.length(vehicles)
        if vehicles[i].propulsion
            push!(loco, vehicles[i])
            deleteat!(vehicles, i)
        end
    end
    Base.length(loco) > 1 ?
    (@warn "The loaded file contains more than one traction unit or multiple unit. Using only the first!") :
    nothing
    loco[1].n > 1 ?
    (@warn "The loaded file contains more than one traction unit or multiple unit. Using only one!") :
    nothing
    Base.length(loco) == 0 ?
    throw(DomainError("No traction unit or multiple unit present in file '$file'")) :
    nothing
    loco = loco[1].data
    cars = vehicles

    ## set the variables for locos
    m_loco = loco["mass"] * 1000
    haskey(loco, "a_braking") ? a_braking = loco["a_braking"] : nothing
    haskey(loco, "base_resistance") ? f_Rtd0 = loco["base_resistance"] : nothing
    haskey(loco, "rolling_resistance") ? f_Rtc0 = loco["rolling_resistance"] : nothing
    haskey(loco, "air_resistance") ? f_Rt2 = loco["air_resistance"] : nothing
    haskey(loco, "mass_traction") ? m_td = loco["mass_traction"] * 1000 : m_td = m_t
    haskey(loco, "rotation_mass") ? ξ_loco = loco["rotation_mass"] : nothing
    m_tc = m_loco - m_td
    haskey(loco, "tractive_effort") ? F_v_pairs = loco["tractive_effort"] :
    F_v_pairs = [[0.0, m_td * g * μ], [v_limit * 3.6, m_td * g * μ]]
    F_v_pairs = reduce(hcat, F_v_pairs)'       # convert to matrix
    F_v_pairs[:, 1] ./= 3.6                    # convert km/h to m/s
    F_v_pairs = tuple.(eachcol(F_v_pairs)...) # convert each row to tuples

    ## set the variables for cars
    if !isempty(cars)
        resis_base = []
        resis_roll = []
        resis_air = []
        rotMassFac = []
        for car in cars
            haskey(car.data, "base_resistance") ?
            append!(resis_base, repeat([car.data["base_resistance"]], car.n)) :
            append!(resis_base, repeat([f_Rw0], car.n))
            haskey(car.data, "rolling_resistance") ?
            append!(resis_roll, repeat([car.data["rolling_resistance"]], car.n)) :
            append!(resis_roll, repeat([f_Rw1], car.n))
            haskey(car.data, "air_resistance") ?
            append!(resis_air, repeat([car.data["air_resistance"]], car.n)) :
            append!(resis_air, repeat([f_Rw2], car.n))
            haskey(car.data, "rotation_mass") ?
            append!(
                rotMassFac,
                repeat([(car.data["rotation_mass"], car.data["mass"])], car.n)
            ) : append!(rotMassFac, repeat([(ξ_cars, car.data["mass"])], car.n))
            m_car_empty += car.data["mass"] * car.n * 1000 # in kg
            m_car_full += car.data["mass"] * car.n * 1000 # in kg
            haskey(car.data, "load_limit") ?
            m_car_full += car.data["load_limit"] * car.n * 1000 :  # in kg
            nothing
        end
        f_Rw0 = Statistics.mean(resis_base)
        f_Rw1 = Statistics.mean(resis_roll)
        f_Rw2 = Statistics.mean(resis_air)
        carRotMass = 0
        for elem in rotMassFac
            carRotMass += elem[1] * elem[2] * 1000 # in kg
        end
        ξ_cars = carRotMass / m_car_empty
        ξ_train = (ξ_loco * m_loco + carRotMass) / m_train_empty
    else
        ξ_cars = 0
        ξ_train = ξ_loco
    end

    Train(
        name,
        id,
        uuid,
        length,
        m_train_full,
        m_td,
        m_tc,
        m_car_full,
        ξ_train,
        ξ_loco,
        ξ_cars,
        transportType,
        v_limit,
        a_braking,
        f_Rtd0,
        f_Rtc0,
        f_Rt2,
        f_Rw0,
        f_Rw1,
        f_Rw2,
        F_v_pairs
    )
end #function Train() # outer constructor

## create a characteristic section for a path section.
function CharacteristicSection(
        s_entry::Real,
        section::Dict,
        v_limit::Real,
        poi_positions::Vector
)
    # Create and return a characteristic section dependent on the paths attributes
    characteristicSection::Dict{Symbol, Any} = Dict(
        :s_entry => s_entry,                    # first position (in m)
        :s_exit => section[:s_end],             # last position  (in m)
        :r_path => section[:f_Rp],              # path resistance (in ‰)
        :v_limit => v_limit,                    # speed limit (in m/s)
        :v_exit => v_limit
    )                     # maximum exit speed (in m/s) initialized with v_limit

    # get the list of positions of every point of interest (POI) in this charateristic section for which support points should be calculated from the list of the whole moving section's POI
    s_exit = characteristicSection[:s_exit]
    CS_poi_positions = Real[]
    for position in poi_positions
        if s_entry <= position && position <= s_exit
            push!(CS_poi_positions, position)
        end
    end
    if isempty(CS_poi_positions) || CS_poi_positions[end] < s_exit
        push!(CS_poi_positions, s_exit)     # s_exit has to be the last POI so that there will always be a POI to campare the current position with
    end
    merge!(characteristicSection, Dict(:pointsOfInterest => CS_poi_positions))

    return characteristicSection
end #function CharacteristicSection

"""
a SupportPoint is the smallest element of the driving course. One step of the step approach is between two support points
"""
function SupportPoint()
    supportPoint = Dict(
        :behavior => "",    # type of behavior section the support point is part of - see BehaviorSection()
        # a support point which is the last point of one behavior section and the first point of the next behavior section will be attached to the latter
        :s => 0.0,          # position (in m)
        :t => 0.0,          # point in time (in s)
        :v => 0.0,          # velocity (in m/s)
        :a => 0.0,          # acceleration (in m/s^2)
        :F_T => 0.0,        # tractive effort (in N)
        :F_R => 0.0,        # resisting force (in N)
        :R_path => 0.0,     # path resistance (in N)
        :R_train => 0.0,    # train resistance (in N)
        :R_traction => 0.0, # traction unit resistance (in N)
        :R_wagons => 0.0   # set of wagons resistance (in N)
    )
    return supportPoint
end #function SupportPoint
