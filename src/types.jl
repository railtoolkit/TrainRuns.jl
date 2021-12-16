module types
# definition of all the additional types and their constructors
export DataPoint, BehaviorSection, CharacteristicSection, EnergySavingModification


## a data point is the smallest element of the driving course. One step of the step approach is between two data points
mutable struct DataPoint
    i::Integer              # identifier and counter variable of the dricing course
    s::AbstractFloat        # position (in m)
    Δs::AbstractFloat       # step size (in m)
    t::AbstractFloat        # point in time (in s)
    Δt::AbstractFloat       # step size (in s)
    v::AbstractFloat        # velocity (in m/s)
    Δv::AbstractFloat       # step size (in m/s)
    a::AbstractFloat        # acceleration (in m/s^2)
    W::AbstractFloat        # mechanical work (in Ws)
    ΔW::AbstractFloat       # mechanical work in this step (in Ws)
    E::AbstractFloat        # energy consumption (in Ws)
    ΔE::AbstractFloat       # energy consumption in this step (in Ws)
    F_T::AbstractFloat      # tractive effort (in N)
    F_R::AbstractFloat      # resisting force (in N)
    R_path::AbstractFloat     # path resistance (in N)
    R_train::AbstractFloat # train resistance (in N)
    R_traction::AbstractFloat     # traction unit resistance (in N)
    R_consist::AbstractFloat     # set of wagons resistance (in N)
end # mutable struct DataPoint
DataPoint()=DataPoint(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# tried to insert copy on 15.07.2021 copy(original::DataPoint)=DataPoint(original.i, original.s, original.Δs, original.t, original.Δt, original.v, original.Δv, original.a, original.W, original.ΔW, original.E, original.ΔE, original.F_T, original.F_R, original.R_path, original.R_train, original.R_traction, original.R_consist)
DataPoint(original::DataPoint)=DataPoint(original.i, original.s, original.Δs, original.t, original.Δt, original.v, original.Δv, original.a, original.W, original.ΔW, original.E, original.ΔE, original.F_T, original.F_R, original.R_path, original.R_train, original.R_traction, original.R_consist)



## different sections the whole path can be devided in the following
## smallest section of the path is the behavior section. It relates to the containing data points via their identifier.
mutable struct BehaviorSection
    type::String                # type of behavior section: "breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "cruisingAfterCoasting","braking" or "standStill"
    length::AbstractFloat       # total length  (in m)
    s_entry::AbstractFloat      # first position (in m)
    s_exit::AbstractFloat       # last position  (in m)
    t::AbstractFloat            # total running time (in s)
    E::AbstractFloat            # total energy consumption (in Ws)
    v_entry::AbstractFloat      # entry speed (in m/s)
    v_exit::AbstractFloat       # exit speed (in m/s)
    dataPoints::Vector{Integer} # list of identifiers of the containing data points
end # mutable struct BehaviorSection
BehaviorSection()=BehaviorSection("", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [])
function BehaviorSection(original::BehaviorSection)
    bsDataPoints=[]
    for i in 1:length(original.dataPoints)
        push!(bsDataPoints, original.dataPoints[i])
    end
    return BehaviorSection(original.type, original.length, original.s_entry, original.s_exit, original.t, original.E, original.v_entry, original.v_exit, bsDataPoints)
end

## a characteristic section is a part of the moving section. It contains behavior sections.
mutable struct CharacteristicSection
    id::Integer                 # identifier
    length::AbstractFloat       # total length  (in m)
    s_entry::AbstractFloat      # first position (in m)
    s_exit::AbstractFloat       # last position  (in m)
    t::AbstractFloat            # total running time (in s)
    E::AbstractFloat            # total energy consumption (in Ws)
    v_limit::AbstractFloat      # speed limit (in m/s)
    v_target::AbstractFloat     # maximum target speed (in m/s)
    v_entry::AbstractFloat      # maximum entry speed (in m/s)
    v_exit::AbstractFloat       # maximum exit speed (in m/s)
    f_Rp::AbstractFloat         # spedific path resistance (in ‰)
    behaviorSections::AbstractDict{Symbol, BehaviorSection}   # list of containing behavior sections
end # mutable struct CharacteristicSection
CharacteristicSection()=CharacteristicSection(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Dict{Symbol, BehaviorSection}())
function CharacteristicSection(original::CharacteristicSection)
    copy=CharacteristicSection(original.id, original.length, original.s_entry, original.s_exit, original.t, original.E, original.v_limit, original.v_target, original.v_entry, original.v_exit, original.f_Rp, Dict{Symbol, BehaviorSection}())
    allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :cruisingAfterCoasting, :braking, :standStill]
    for bs in 1: length(allBs)
        if haskey(original.behaviorSections, allBs[bs])
            merge!(copy.behaviorSections, Dict(allBs[bs] => BehaviorSection(original.behaviorSections[allBs[bs]])))
        end #if
    end #for
    return copy
end #function CharacteristicSection

## for the energy saving operation mode it is nesserary to compare different energy saving modifications. These are part of the moving section.
mutable struct EnergySavingModification
    csId::Integer                               # identifier of the characteristic section
    type::String                                # type of energy saving modification: "increasing coasting" "decreasing maximum velocity" or "combination of decreasing maximum velocity and coasting"
    ΔE::AbstractFloat                           # saved energy (in Ws)
    Δt::AbstractFloat                           # time loss (in s)
    ratio::AbstractFloat                        # ratio of ΔE and Δt (in Ws/s)
    csModified::CharacteristicSection           # the modified characteristic section
    drivingCourseModified::Vector{DataPoint}    # drivingCourse for the modified characteristic section
end # mutable struct EnergySavingModification
EnergySavingModification()=EnergySavingModification(0, "", 0.0, 0.0, 0.0, CharacteristicSection(), [])
function EnergySavingModification(original::EnergySavingModification)
    copy=EnergySavingModification(original.csId, original.type, original.ΔE, original.Δt, original.ratio, CharacteristicSection(), DataPoint[])
    copy.csModified=CharacteristicSection(original.csModified)
    for i in 1:length(original.drivingCourseModified)
        push!(copy.drivingCourseModified, DataPoint(original.drivingCourseModified[i]))
    end
    return copy
end #function EnergySavingModification

end #module
