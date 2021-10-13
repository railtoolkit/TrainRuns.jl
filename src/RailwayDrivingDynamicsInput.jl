module RailwayDrivingDynamicsInput

import YAML
using ..types

export readInput

function readInput(vehicleFilePath::String, pathFilePath::String, settingsFilePath::String)
 # this function reads the input information from YAML files, saves it in a different objects and returns them
     vehicle=inputVehicle(vehicleFilePath)
     path=inputPath(pathFilePath)
     settings=inputSettings(settingsFilePath)

     return (vehicle, path, settings)
 end #function readInput


function inputVehicle(vehicleFilePath::String)
 # this function reads vehicle information from a YAML file, saves it in a Vehicle object and returns it
    data = YAML.load(open(vehicleFilePath))
    collect(keys(data))
    collect(values(data))

    vehicle=Vehicle()
    if haskey(data["vehicle"],"name")
        vehicle.name=data["vehicle"]["name"]                            # vehicles name
        delete!(data["vehicle"], "name")
    else
        error("ERROR at reading the vehicle yaml file: The keyword name is missing. It has to be added.")
    end
    vehicle.id=1                                # vehicles identifier

    if haskey(data["vehicle"],"vehicleType")
            if typeof(data["vehicle"]["vehicleType"])==String && (data["vehicle"]["vehicleType"]=="freight" || data["vehicle"]["vehicleType"]=="motor coach train" || data["vehicle"]["vehicleType"]=="passenger")
            vehicle.vehicleType=data["vehicle"]["vehicleType"]                  # "passenger" or "freight" or "motor coach train"
            delete!(data["vehicle"], "vehicleType")
        else
            error("ERROR at reading the vehicle yaml file: The value of vehicleType is wrong. It has to be freight, motor coach train or passenger.")
        end
    else
        error("ERROR at reading the vehicle yaml file: The keyword vehicleType is missing. It has to be added with the value freight, motor coach train or passenger.")
    end

    if haskey(data["vehicle"],"l_union")
        if typeof(data["vehicle"]["l_union"])==Float64 && data["vehicle"]["l_union"]>0.0
            vehicle.l_union=data["vehicle"]["l_union"]                                                # total length (in m)
            delete!(data["vehicle"], "l_union")
        elseif typeof(data["vehicle"]["l_union"])==Int64 && data["vehicle"]["l_union"]>0
            vehicle.l_union=convert(AbstractFloat, data["vehicle"]["l_union"])                        # total length (in m)
            delete!(data["vehicle"], "l_union")
        else
            error("ERROR at reading the vehicle yaml file: The value of the length is no real floating point number >0.0.")
        end
    else
        error("ERROR at reading the vehicle yaml file: The keyword length is missing. It has to be added with a value of type real floating point number >0.0.")
    end

    # speed limit:                                                                      # vehicles speed limit (in m/s)
    if haskey(data["vehicle"],"v_limit") && haskey(data["vehicle"],"v_limit_kmh")
        if data["vehicle"]["v_limit"]!=nothing
            if typeof(data["vehicle"]["v_limit"])==Float64 && data["vehicle"]["v_limit"]>0.0
                vehicle.v_limit=data["vehicle"]["v_limit"]                # vehicles speed limit (in m/s)
                delete!(data["vehicle"], "v_limit")
            elseif typeof(data["vehicle"]["v_limit"])==Int64 && data["vehicle"]["v_limit"]>0
                vehicle.v_limit=convert(AbstractFloat, data["vehicle"]["v_limit"])                   # vehicles speed limit (in m/s)
                delete!(data["vehicle"], "v_limit")
            else
                error("ERROR at reading the vehicle yaml file: The value of v_limit is no real floating point number >0.0.")
            end

            if data["vehicle"]["v_limit_kmh"]!=nothing
                v_limit_kmh=data["vehicle"]["v_limit_kmh"]
                if abs(vehicle.v_limit-v_limit_kmh/3.6)>0.001
                    println("WARNING: at reading the vehicle yaml file: The values of v_limit and v_limit_kmh differ by more than 0.001 m/s. The value v_limit=",vehicle.v_limit," m/s is used." )
                end
            end

        elseif data["vehicle"]["v_limit_kmh"]!=nothing
            if typeof(data["vehicle"]["v_limit_kmh"])==Float64 && data["vehicle"]["v_limit_kmh"]>0.0
                vehicle.v_limit=data["vehicle"]["v_limit_kmh"]/3.6                                               # vehicles speed limit (from km/h to m/s)
            elseif typeof(data["vehicle"]["v_limit_kmh"])==Int64 && data["vehicle"]["v_limit_kmh"]>0
                vehicle.v_limit=convert(AbstractFloat, data["vehicle"]["v_limit_kmh"])/3.6                          # vehicles speed limit (from km/h to m/s)
            else
                error("ERROR at reading the vehicle yaml file: The value of the v_limit_kmh is no real floating point number >0.0.")
            end
        else
            vehicle.v_limit=1000/3.6
            println("WARNING at reading the vehicle yaml file: There is no value for v_limit. The value v_limit=1000 km/h =",vehicle.v_limit," m/s is used." )
        end # if
    elseif haskey(data["vehicle"],"v_limit")
        if typeof(data["vehicle"]["v_limit"])==Float64 && data["vehicle"]["v_limit"]>0.0
            vehicle.v_limit=data["vehicle"]["v_limit"]                                                    # vehicles speed limit (in m/s)
        elseif typeof(data["vehicle"]["v_limit"])==Int64 && data["vehicle"]["v_limit"]>0
            vehicle.v_limit=convert(AbstractFloat, data["vehicle"]["v_limit"])                               # vehicles speed limit (in m/s)
        else
            error("ERROR at reading the vehicle yaml file: The value of v_limit is no real floating point number >0.0.")
        end
    elseif haskey(data["vehicle"],"v_limit_kmh")
        if typeof(data["vehicle"]["v_limit_kmh"])==Float64 && data["vehicle"]["v_limit_kmh"]>0.0
            vehicle.v_limit=data["vehicle"]["v_limit_kmh"]/3.6                                               # vehicles speed limit (from km/h to m/s)
        elseif typeof(data["vehicle"]["v_limit_kmh"])==Int64 && data["vehicle"]["v_limit_kmh"]>0
            vehicle.v_limit=convert(AbstractFloat, data["vehicle"]["v_limit_kmh"])/3.6                          # vehicles speed limit (from km/h to m/s)
        else
            error("ERROR at reading the vehicle yaml file: The value of v_limit_kmh is no real floating point number >0.0.")
        end
    else
        error("ERROR at reading the vehicle yaml file: The keyword v_limit or v_limit_kmh is missing. It has to be added with a value of type real floating point number >0.0.")
    end
    delete!(data["vehicle"], "v_limit")
    delete!(data["vehicle"], "v_limit_kmh")


    # a_braking
    if haskey(data["vehicle"],"a_braking")
        if typeof(data["vehicle"]["a_braking"])==Float64
            vehicle.a_braking=data["vehicle"]["a_braking"]
        elseif typeof(data["vehicle"]["a_braking"])==Int64
            vehicle.a_braking=convert(AbstractFloat, data["vehicle"]["a_braking"])
        else
            error("ERROR at reading the vehicle yaml file: The value of the a_braking is no real floating point number <0.0.")
        end
        delete!(data["vehicle"], "a_braking")

        if vehicle.a_braking > 0.0
            vehicle.a_braking =-vehicle.a_braking
            println("WARNING at reading the vehicle yaml file: The value for a_braking is >0.0. The braking acceleration has to be <0.0. Therefore a_braking =",vehicle.a_braking," m/s^2 is used." )
        elseif vehicle.a_braking == 0.0
            error("ERROR at reading the vehicle yaml file: The value for a_braking is 0.0. The braking acceleration has to be <0.0.")
        end
    else
        error("ERROR at reading the vehicle yaml file: The keyword a_braking is missing. It has to be added with a value of type real floating point number <0.0.")
    end

    # mass on the traction units driving axles (in kg)
    if haskey(data["vehicle"],"m_td")
        if typeof(data["vehicle"]["m_td"])==Float64 && data["vehicle"]["m_td"]>0.0
            vehicle.m_td=data["vehicle"]["m_td"]
        elseif typeof(data["vehicle"]["m_td"])==Int64 && data["vehicle"]["m_td"]>0
            vehicle.m_td=convert(AbstractFloat, data["vehicle"]["m_td"])
            delete!(data["vehicle"], "m_td")
        else
            error("ERROR at reading the vehicle yaml file: The value of m_td is no real floating point number >0.0.")
        end
    else
        error("ERROR at reading the vehicle yaml file: The keyword m_td is missing. It has to be added with a value of type real floating point number >0.0.")
    end
    delete!(data["vehicle"], "m_td")


    # mass on the traction units carrying axles (in kg)
    if haskey(data["vehicle"],"m_tc")
        if typeof(data["vehicle"]["m_tc"])==Float64 && data["vehicle"]["m_tc"]>=0.0
            vehicle.m_tc=data["vehicle"]["m_tc"]
        elseif typeof(data["vehicle"]["m_tc"])==Int64 && data["vehicle"]["m_tc"]>=0
            vehicle.m_tc=convert(AbstractFloat, data["vehicle"]["m_tc"])
        else
            error("ERROR at reading the vehicle yaml file: The value of m_tc is no real floating point number >=0.0.")
        end
    else
        error("ERROR at reading the vehicle yaml file: The keyword m_tc is missing. It has to be added with a value of type real floating point number >=0.0.")
    end
    delete!(data["vehicle"], "m_tc")

    # mass of the traction unit (in kg)
    vehicle.m_t=vehicle.m_td+vehicle.m_tc


    # mass of theconsist (set of wagons)  (in kg)
    if haskey(data["vehicle"],"m_w")
        if typeof(data["vehicle"]["m_w"])==Float64 && data["vehicle"]["m_w"]>=0.0
            vehicle.m_w=data["vehicle"]["m_w"]
        elseif typeof(data["vehicle"]["m_w"])==Int64 && data["vehicle"]["m_w"]>=0
            vehicle.m_w=convert(AbstractFloat, data["vehicle"]["m_w"])
        else
            error("ERROR at reading the vehicle yaml file: The value of m_w is no real floating point number >=0.0.")
        end
    else
        vehicle.m_w=0.0
        println("WARNING at reading the vehicle yaml file: The keyword m_w is missing. Therefore m_w =",vehicle.m_w," kg is used.")
    end
    delete!(data["vehicle"], "m_w")

    # total mass (in kg)
    vehicle.m_union=vehicle.m_t+vehicle.m_w

    if haskey(data["vehicle"],"rotationMassFactor_union") && (typeof(data["vehicle"]["rotationMassFactor_union"])==Float64 || typeof(data["vehicle"]["rotationMassFactor_union"])==Int64)
        if typeof(data["vehicle"]["rotationMassFactor_union"])==Float64 && data["vehicle"]["rotationMassFactor_union"]>0.0
            vehicle.ξ_union=data["vehicle"]["rotationMassFactor_union"]
        elseif typeof(data["vehicle"]["rotationMassFactor_union"])==Int64 && data["vehicle"]["rotationMassFactor_union"]>0
            vehicle.ξ_union=convert(AbstractFloat, data["vehicle"]["rotationMassFactor_union"])
        else
            error("ERROR at reading the vehicle yaml file: The value of rotationMassFactor_union is no real floating point number >0.0.")
        end
    elseif haskey(data["vehicle"],"rotationMassFactor_t") && (typeof(data["vehicle"]["rotationMassFactor_t"])==Float64 || typeof(data["vehicle"]["rotationMassFactor_t"])==Int64) && (vehicle.m_w==0.0 || haskey(data["vehicle"],"rotationMassFactor_w") && (typeof(data["vehicle"]["rotationMassFactor_w"])==Float64 || typeof(data["vehicle"]["rotationMassFactor_w"])==Int64))
        if typeof(data["vehicle"]["rotationMassFactor_t"])==Float64 && data["vehicle"]["rotationMassFactor_t"]>0.0
            vehicle.ξ_t=data["vehicle"]["rotationMassFactor_t"]
        elseif typeof(data["vehicle"]["rotationMassFactor_t"])==Int64 && data["vehicle"]["rotationMassFactor_t"]>0
            vehicle.ξ_t=convert(AbstractFloat, data["vehicle"]["rotationMassFactor_t"])
        else
            error("ERROR at reading the vehicle yaml file: The value of rotationMassFactor_t is no real floating point number >0.0.")
        end
        if vehicle.m_w>0.0
            if typeof(data["vehicle"]["rotationMassFactor_w"])==Float64 && data["vehicle"]["rotationMassFactor_w"]>=0.0
                vehicle.ξ_w=data["vehicle"]["rotationMassFactor_w"]
            elseif typeof(data["vehicle"]["rotationMassFactor_w"])==Int64 && data["vehicle"]["rotationMassFactor_w"]>=0
                vehicle.ξ_w=convert(AbstractFloat, data["vehicle"]["rotationMassFactor_w"])
            else
                error("ERROR at reading the vehicle yaml file: The value of rotationMassFactor_w is no real floating point number >=0.0.")
            end
        else
            vehicle.ξ_w=0.0
        end
        vehicle.ξ_union=(vehicle.ξ_t*vehicle.m_t + vehicle.ξ_w*vehicle.m_w)/vehicle.m_union  # rotation mass factor of the whole vehicle (without unit)
    else
        error("ERROR at reading the vehicle yaml file: The keywords rotationMassFactor_union or rotationMassFactor_t and rotationMassFactor_w are missing. They has to be added with a value of type real floating point number.")
    end
    delete!(data["vehicle"], "rotationMassFactor_union")
    delete!(data["vehicle"], "rotationMassFactor_t")
    delete!(data["vehicle"], "rotationMassFactor_w")


    # input for function tractiveEffort(v)
        # TODO: The types of the lists elements can be checked and the user can get warnings or error messages
        # TODO: What if there are values in F_T_pairs and in F_T_pairs_kmh. That should be checked and the user should be informed
        # TODO: It could be checked if all the array elements are correct.
    if haskey(data["vehicle"],"F_T_pairs") && data["vehicle"]["F_T_pairs"]!=nothing
        F_T_pairs=data["vehicle"]["F_T_pairs"]
        vehicle.tractiveEffortArray=[]
        if F_T_pairs[1][1]==0.0
            push!(vehicle.tractiveEffortArray, [F_T_pairs[1][1], F_T_pairs[1][1], F_T_pairs[1][2]])
        elseif F_T_pairs[1][1]>0.0    # if there is no F_T for v=0.0, the first value is used
            push!(vehicle.tractiveEffortArray, [0.0, F_T_pairs[1][1]/3.6, F_T_pairs[1][2]])
            println("WARNING at reading the vehicle yaml file: The tractive effort for v=0 m/s is missing. Therefore the first given value F_T(v=",F_T_pairs[1][1]," m/s)=",F_T_pairs[1][2]," N will be used." )
        else
            error("ERROR at reading the vehicle yaml file: There is a negative speed value in the list. Only positive values for speed and tractive effort are allowed. ")
        end
        for row in 2:length(F_T_pairs)
            if F_T_pairs[row][1]>F_T_pairs[row-1][1]
                if F_T_pairs[row][2]==vehicle.tractiveEffortArray[end][3]
                    vehicle.tractiveEffortArray[end][2]=F_T_pairs[row][1]
                else
                    push!(vehicle.tractiveEffortArray, [F_T_pairs[row][1], F_T_pairs[row][1], F_T_pairs[row][2]])
                end
            else
                error("ERROR at reading the vehicle yaml file: The F_T_pairs are not in the correct order. They have to be arranged by speed values from low to high.")
            end
        end # for
        if length(F_T_pairs[1])>2
            println("INFO according the vehicle yaml file: Only the first two columns of F_T_pairs are used in this tool.")
        end


    elseif haskey(data["vehicle"],"F_T_pairs_kmh") && data["vehicle"]["F_T_pairs_kmh"]!=nothing
        F_T_pairs_kmh=data["vehicle"]["F_T_pairs_kmh"]
        vehicle.tractiveEffortArray=[]
        if F_T_pairs_kmh[1][1]==0.0
            push!(vehicle.tractiveEffortArray, [F_T_pairs_kmh[1][1]/3.6, F_T_pairs_kmh[1][1]/3.6, F_T_pairs_kmh[1][2]])
        elseif F_T_pairs_kmh[1][1]>0.0    # if there is no F_T for v=0.0, the first value is used
            push!(vehicle.tractiveEffortArray, [0.0, F_T_pairs_kmh[1][1]/3.6, F_T_pairs_kmh[1][2]])
            println("WARNING at reading the vehicle yaml file: The tractive effort for v=0 km/h is missing. Therefore the first given value F_T(v=",F_T_pairs_kmh[1][1]," km/h)=",F_T_pairs_kmh[1][2]," N will be used." )
        else
            error("ERROR at reading the vehicle yaml file: There is a negative speed value in the list. Only positive values for speed and tractive effort are allowed. ")
        end
        for row in 2:length(F_T_pairs_kmh)
            if F_T_pairs_kmh[row][1]>F_T_pairs_kmh[row-1][1]
                if F_T_pairs_kmh[row][2]==vehicle.tractiveEffortArray[end][3]
                    vehicle.tractiveEffortArray[end][2]=F_T_pairs_kmh[row][1]/3.6
                else
                    push!(vehicle.tractiveEffortArray, [F_T_pairs_kmh[row][1]/3.6, F_T_pairs_kmh[row][1]/3.6, F_T_pairs_kmh[row][2]])
                end
            else
                error("ERROR at reading the vehicle yaml file: The F_T_pairs_kmh are not in the correct order. They have to be arranged by speed values from low to high.")
            end
        end # for
        if length(F_T_pairs_kmh[1])>2
            println("INFO at reading the vehicle yaml file: Only the first two columns of F_T_pairs_kmh are used in this tool.")
        end
    else
        error("ERROR at reading the vehicle yaml file: There has to be one of the keywords F_T_pairs or F_T_pairs_kmh filled with a list of pairs of velocity and tractive effort.")
    end # if
    delete!(data["vehicle"], "F_T_pairs")
    delete!(data["vehicle"], "F_T_pairs_kmh")


    # coefficients for the vehicle resistance of the traction unit
    vehicle.Δv_t=15.0/3.6                        # coefficient for velocitiy difference between traction unit and outdoor air

    # coefficient for basic resistance due to the traction units driving axles (in ‰)
    if haskey(data["vehicle"],"f_Rtd0") && data["vehicle"]["f_Rtd0"]!=nothing
        if typeof(data["vehicle"]["f_Rtd0"])==Float64 && data["vehicle"]["f_Rtd0"]>=0.0
            vehicle.f_Rtd0=data["vehicle"]["f_Rtd0"]
        elseif typeof(data["vehicle"]["f_Rtd0"])==Int64 && data["vehicle"]["f_Rtd0"]>=0
            vehicle.f_Rtd0=convert(AbstractFloat, data["vehicle"]["f_Rtd0"])
        else
            error("ERROR at reading the vehicle yaml file: The value of f_Rtd0 is no real floating point number >=0.0.")
        end
    else
        vehicle.f_Rtd0=0.0
        println("WARNING at reading the vehicle yaml file: The keyword f_Rtd0 is missing. Therefore f_Rtd0=0.0 ‰ will be assumed and used." )
    end
    delete!(data["vehicle"], "f_Rtd0")

    # coefficient for basic resistance due to the traction units carring axles (in ‰)
    if haskey(data["vehicle"],"f_Rtc0") && data["vehicle"]["f_Rtc0"]!=nothing
        if typeof(data["vehicle"]["f_Rtc0"])==Float64 && data["vehicle"]["f_Rtc0"]>=0.0
            vehicle.f_Rtc0=data["vehicle"]["f_Rtc0"]
        elseif typeof(data["vehicle"]["f_Rtc0"])==Int64 && data["vehicle"]["f_Rtc0"]>=0
            vehicle.f_Rtc0=convert(AbstractFloat, data["vehicle"]["f_Rtc0"])
        else
            error("ERROR at reading the vehicle yaml file: The value of f_Rtc0 is no real floating point number >=0.0.")
        end
    else
        vehicle.f_Rtc0=0.0
        println("WARNING at reading the vehicle yaml file: The keyword f_Rtc0 is missing. Therefore f_Rtc0=0.0 ‰ will be assumed and used." )
    end
    delete!(data["vehicle"], "f_Rtc0")

    # coefficient for air resistance of the traction units (in N)
    if haskey(data["vehicle"],"F_Rt2") && data["vehicle"]["F_Rt2"]!=nothing
        if typeof(data["vehicle"]["F_Rt2"])==Float64 && data["vehicle"]["F_Rt2"]>=0.0
            vehicle.F_Rt2=data["vehicle"]["F_Rt2"]
        elseif typeof(data["vehicle"]["F_Rt2"])==Int64 && data["vehicle"]["F_Rt2"]>=0
            vehicle.F_Rt2=convert(AbstractFloat, data["vehicle"]["F_Rt2"])
        else
            error("ERROR at reading the vehicle yaml file: The value of F_Rt2 is no real floating point number >=0.0.")
        end
    else
        vehicle.F_Rt2=0.0
        println("WARNING at reading the vehicle yaml file: The keyword F_Rt2 is missing. Therefore F_Rt2=0.0 N will be assumed and used." )
    end
    delete!(data["vehicle"], "F_Rt2")



    # coefficients for the vehicle resistance of the consist (set of wagons)
    # coefficient for velocitiy difference betweenconsist (set of wagons)  and outdoor air (in m/s)
    if vehicle.vehicleType=="passenger" || vehicle.vehicleType=="motor coach train"
        vehicle.Δv_w=15.0/3.6
    elseif vehicle.vehicleType== "freight"
        vehicle.Δv_w=0.0
    end # if

    # coefficient for basic resistance of theconsist (set of wagons)  (in ‰)
    if haskey(data["vehicle"],"f_Rw0") && data["vehicle"]["f_Rw0"]!=nothing
        if typeof(data["vehicle"]["f_Rw0"])==Float64 && data["vehicle"]["f_Rw0"]>=0.0
            vehicle.f_Rw0=data["vehicle"]["f_Rw0"]
        elseif typeof(data["vehicle"]["f_Rw0"])==Int64 && data["vehicle"]["f_Rw0"]>=0
            vehicle.f_Rw0=convert(AbstractFloat, data["vehicle"]["f_Rw0"])
        else
            error("ERROR at reading the vehicle yaml file: The value of f_Rw0 is no real floating point number >=0.0.")
        end
    else
        vehicle.f_Rw0=0.0
        println("WARNING at reading the vehicle yaml file: The keyword f_Rw0 is missing. Therefore f_Rw0=0.0 ‰ will be assumed and used." )
    end
    delete!(data["vehicle"], "f_Rw0")

    # coefficient for basic resistance of theconsist (set of wagons)  (in ‰)
    if haskey(data["vehicle"],"f_Rw1") && data["vehicle"]["f_Rw1"]!=nothing
        if typeof(data["vehicle"]["f_Rw1"])==Float64 && data["vehicle"]["f_Rw1"]>=0.0
            vehicle.f_Rw1=data["vehicle"]["f_Rw1"]
        elseif typeof(data["vehicle"]["f_Rw1"])==Int64 && data["vehicle"]["f_Rw1"]>=0
            vehicle.f_Rw1=convert(AbstractFloat, data["vehicle"]["f_Rw1"])
        else
            error("ERROR at reading the vehicle yaml file: The value of f_Rw1 is no real floating point number >=0.0.")
        end
    else
        vehicle.f_Rw1=0.0
        println("WARNING at reading the vehicle yaml file: The keyword f_Rw1 is missing. Therefore f_Rw1=0.0 ‰ will be assumed and used." )
    end
    delete!(data["vehicle"], "f_Rw1")

    # coefficient for basic resistance of the consist (set of wagons) (in ‰)
    if haskey(data["vehicle"],"f_Rw2") && data["vehicle"]["f_Rw2"]!=nothing
        if typeof(data["vehicle"]["f_Rw2"])==Float64 && data["vehicle"]["f_Rw2"]>=0.0
            vehicle.f_Rw2=data["vehicle"]["f_Rw2"]
        elseif typeof(data["vehicle"]["f_Rw2"])==Int64 && data["vehicle"]["f_Rw2"]>=0
            vehicle.f_Rw2=convert(AbstractFloat, data["vehicle"]["f_Rw2"])
        else
            error("ERROR at reading the vehicle yaml file: The value of f_Rw2 is no real floating point number >=0.0.")
        end
    else
        vehicle.f_Rw2=0.0
        println("WARNING at reading the vehicle yaml file: The keyword f_Rw2 is missing. Therefore f_Rw2=0.0 ‰ will be assumed and used." )
    end
    delete!(data["vehicle"], "f_Rw2")


    # informing the user, which keywords of the input data are not used in this tool:
    if length(data["vehicle"])>0
        println("INFO at reading the vehicle yaml file: The following Keywords are not used in this tool:")
        for key in keys(data["vehicle"])
            println("  - ",key)
        end
    end

    return vehicle
end #function inputVehicle


function inputPath(pathFilePath::String)
 # this function reads path information from a YAML file, saves it in a Path object and returns it
    data = YAML.load(open(pathFilePath))
    collect(keys(data))
    collect(values(data))


    if haskey(data["path"],"name") && data["path"]["name"]!=nothing
        name=data["path"]["name"]                            # path name
    else
        error("ERROR at reading the path yaml file: The keyword name is missing. It has to be added.")
    end
    delete!(data["path"], "name")

    id=1                                                    # path identifier
    path=Path(name, id, [])


    # TODO: It could be checked if all the array elements are correct.
    # TODO: The types of the lists elements can be checked and the user can get warnings or error messages
    # TODO: What if there are values in sectionStarts and in sectionStarts_kmh? The user should be informed
    if haskey(data["path"],"sectionStarts") && data["path"]["sectionStarts"]!=nothing
        sectionStartsArray=data["path"]["sectionStarts"]
        conversionFactor=1.0                                # conversion factor between the units m/s and m/s
    elseif haskey(data["path"],"sectionStarts_kmh") && data["path"]["sectionStarts_kmh"]!=nothing
        sectionStartsArray=data["path"]["sectionStarts_kmh"]
        conversionFactor=1/3.6                              # conversion factor between the units km/h and m/s
    else
        error("ERROR at reading the path yaml file: The keyword sectionStarts oder sectionStarts_kmh is missing. It has to be added.")
    end # if
    delete!(data["path"], "sectionStarts")
    delete!(data["path"], "sectionStarts_kmh")

    if length(sectionStartsArray)>=2
        if length(sectionStartsArray[1])>=3
            for row in 2:length(sectionStartsArray)
                s_start=sectionStartsArray[row-1][1]         # starting point of the section (in m)
                s_startNext=sectionStartsArray[row][1]       # starting point of the next section (in m)
                if s_startNext<s_start
                    error("ERROR at reading the path yaml file: The keyword sectionStartsArray needs to be filled with the three columns [s, v_limit, gradient].")
                end
                v_limit=sectionStartsArray[row-1][2]*conversionFactor      # paths speed limt (in m/s)
                n=sectionStartsArray[row-1][3]              # gradient (in ‰)
                f_Rp=n                                       # specific path resistance of the section (in ‰)
                section=PathSection(s_start, s_startNext, v_limit, n, f_Rp)
                push!(path.sections, section)
            end # for
            if length(sectionStartsArray[1])>3
                println("INFO at reading the path yaml file: Only the first three columns of sectionStartsArray are used in this tool.")
            end
        else
            error("ERROR at reading the path yaml file: The keyword sectionStartsArray needs to be filled with the three columns [s, v_limit, gradient].")
        end
    else
        error("ERROR at reading the path yaml file: The keyword sectionStartsArray needs at least two rows for two points each with the three columns [s, v_limit, gradient].")
    end

    # informing the user, which keywords of the input data are not used in this tool:
    if length(data["path"])>0
        println("INFO at reading the path yaml file: The following Keywords are not used in this tool:")
        for key in keys(data["path"])
            println("  - ",key)
        end
    end
    return path
end # function inputPath




function inputSettings(settingsFilePath::String)
 # this function reads setting information from a YAML file, saves it in a Setting object and returns it
    data = YAML.load(open(settingsFilePath))
    collect(keys(data))
    collect(values(data))

    settings=Settings()

    # model type of the unions mass "mass point" or "homogeneous strip"
    if haskey(data["settings"],"massModel")
            if typeof(data["settings"]["massModel"])==String && (data["settings"]["massModel"]=="mass point" || data["settings"]["massModel"]=="homogeneous strip")
            settings.massModel=data["settings"]["massModel"]                  # "mass point" or "homogeneous strip"
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
            settings.stepVariable=data["settings"]["stepVariable"]                  # "s in m", "t in s" or "v in m/s"
        else
            error("ERROR at reading the settings yaml file: The value of stepVariable is wrong. It has to be s in m, t in s or v in m/s.")
        end
    else
        error("ERROR for the settings yaml file: The keyword stepVariable is missing. It has to be added with the value s in m, t in s or v in m/s.")
    end
    delete!(data["settings"], "stepVariable")


     # step size (unit depends on steapVariable: s in m, t in s and v in m/s)
    if haskey(data["settings"],"stepSize")
        if typeof(data["settings"]["stepSize"])==Float64 && data["settings"]["stepSize"]>0.0
            settings.stepSize=data["settings"]["stepSize"]
        elseif typeof(data["settings"]["stepSize"])==Int64 && data["settings"]["stepSize"]>0
            settings.stepSize=convert(AbstractFloat, data["settings"]["stepSize"])
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
            settings.operationModeMinimumRunningTime=data["settings"]["operationModeMinimumRunningTime"]
        else
            error("ERROR at reading the settings yaml file: The value of the keyword operationModeMinimumRunningTime is not correct. The value has to be of type boolean.")
        end
    else
        settings.operationModeMinimumRunningTime=false
        println("WARNING at reading the settings yaml file: The keyword operationModeMinimumRunningTime or its value is missing. Therefore operationModeMinimumRunningTime=",settings.operationModeMinimumRunningTime," is assumed and used.")
    end
    delete!(data["settings"], "operationModeMinimumRunningTime")


    # operation mode "minimum energy consumption"
    if haskey(data["settings"],"operationModeMinimumEnergyConsumption") && data["settings"]["operationModeMinimumEnergyConsumption"]!=nothing
        if typeof(data["settings"]["operationModeMinimumEnergyConsumption"])==Bool
            settings.operationModeMinimumEnergyConsumption=data["settings"]["operationModeMinimumEnergyConsumption"]
        else
            error("ERROR at reading the settings yaml file: The value of the keyword operationModeMinimumEnergyConsumption is not correct. The value has to be of type boolean.")
        end
    else
        settings.operationModeMinimumEnergyConsumption=false
        println("WARNING at reading the settings yaml file: The keyword operationModeMinimumEnergyConsumption or its value is missing. Therefore operationModeMinimumEnergyConsumption=",settings.operationModeMinimumEnergyConsumption," is assumed and used.")
    end
    delete!(data["settings"], "operationModeMinimumEnergyConsumption")

    # output as "julia dictionary" or as "CSV"
    if haskey(data["settings"],"typeOfOutput")
        if typeof(data["settings"]["typeOfOutput"])==String && (data["settings"]["typeOfOutput"]=="julia dictionary" || data["settings"]["typeOfOutput"]=="CSV")
            settings.typeOfOutput=data["settings"]["typeOfOutput"]                  # "julia dictionary" or "CSV"
        else
            error("ERROR at reading the settings yaml file: The value of typeOfOutput is wrong. It has to be julia dictionary or CSV.")
        end
    else
        error("ERROR at reading the settings yaml file: The keyword typeOfOutput is missing. It has to be added with the value julia dictionary or CSV.")
    end
    delete!(data["settings"], "typeOfOutput")

    # TODO: it could be checked if the path is existing on the pc
    if settings.typeOfOutput=="CSV"
        if haskey(data["settings"],"csvFolderPath")
                if typeof(data["settings"]["csvFolderPath"])==String
                settings.csvFolderPath=data["settings"]["csvFolderPath"]
            else
                error("ERROR at reading the settings yaml file: The value of csvFolderPath is wrong. It has to be of type String.")
            end
        else
            error("ERROR at reading the settings yaml file: The keyword csvFolderPath is missing. It has to be added.")
        end
        delete!(data["settings"], "csvFolderPath")
    end # if


    # should the output be "reduced" or "driving course"
    if haskey(data["settings"],"detailOfOutput")
            if typeof(data["settings"]["detailOfOutput"])==String && (data["settings"]["detailOfOutput"]=="reduced" || data["settings"]["detailOfOutput"]=="driving course")
            settings.detailOfOutput=data["settings"]["detailOfOutput"]                  # "reduced" or "driving course"
        else
            error("ERROR at reading the settings yaml file: The value of detailOfOutput is wrong. It has to be reduced or driving course.")
        end
    else
        error("ERROR at reading the settings yaml file: The keyword detailOfOutput is missing. It has to be added with the value reduced or driving course.")
    end
    delete!(data["settings"], "detailOfOutput")


    # informing the user, which keywords of the input data are not used in this tool:
    if length(data["settings"])>0
        println("INFO at reading the settings yaml file: The following Keywords are not used in this tool:")
        for key in keys(data["settings"])
            println("  - ",key)
        end
    end

    return settings
end # function inputSettings

end # module RailwayDrivingDynamicsInput
