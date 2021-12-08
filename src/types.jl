module types
# definition of all the additional types and their constructors
export Settings, Train, PathSection, Path, DataPoint, BehaviorSection, CharacteristicSection, EnergySavingModification, MovingSection # tried to insert copy on 15.07.2021 , copy

## settings for the simulation
mutable struct Settings
    massModel::String           # model type of the unions mass "mass point" or "homogeneous strip"
    stepVariable::String        # step variable of the step method "s in m", "t in s" or "v in m/s"
    stepSize::AbstractFloat #    step size (unit depends on steapVariable s in m, t in s and v in m/s)
    operationModeMinimumRunningTime::Bool           # operation mode "minimum running time"
    operationModeMinimumEnergyConsumption::Bool     # operation mode "minimum energy consumption"
    # output:
    typeOfOutput::String        # output as "julia dictionary" or as "CSV"
    csvFolderPath::String       # path of the folder in which the CSV fiiles willl be saved
    detailOfOutput::String      # detail of output "reduced" or "everything"
end # mutable struct Settings
Settings()=Settings("", "", 0.0, false, false, "", "", "")


## train
mutable struct Train
    name::String                # trains name
    id                          # trains identifier
    trainType::String         # type of train "passenger" or "freight" or "motor coach train"
    l_union::AbstractFloat      # total length (in m)
    v_limit::AbstractFloat      # trains speed limit (in m/s)
    a_braking::AbstractFloat    # braking acceleration (in m/s^2)
    m_union::AbstractFloat      # total mass (in kg)
    ξ_union::AbstractFloat      # rotation mass factor of the whole train union (without unit)
                                # if not available use ξ_t and ξ_w

    # traction unit
    m_t::AbstractFloat          # mass of the traction unit (in kg)
    m_td::AbstractFloat         # mass on the traction units driving axles (in kg)
    m_tc::AbstractFloat         # mass on the traction units carrying axles (in kg)
    ξ_t::AbstractFloat          # rotation mass factor of the traction unit (without unit)
                                # in case ξ_union is not available
    tractiveEffortArray         # list values for tractive effort (in [m/s , N])

    f_Rtd0::AbstractFloat       # coefficient for basic resistance due to the traction units driving axles (in ‰)
    f_Rtc0::AbstractFloat       # coefficient for basic resistance due to the traction units carring axles (in ‰)
    F_Rt2::AbstractFloat #todo: war mal:forceRT2        # coefficient for air resistance of the traction units (in N)
    Δv_t::AbstractFloat         # coefficient for velocitiy difference between traction unit and outdoor air


    # set of wagons
    m_w::AbstractFloat          # mass of the set of wagons (in kg)
    ξ_w::AbstractFloat          # rotation mass factor of the set of wagons (without unit)
                                # in case ξ_union is not available
    f_Rw0::AbstractFloat        # coefficient for basic resistance of the set of wagons (in ‰)
    f_Rw1::AbstractFloat        # coefficient for resistance to rolling of the set of wagons (in ‰)
    f_Rw2::AbstractFloat        # coefficient for air resistance of the set of wagons (in ‰)
    Δv_w::AbstractFloat         # coefficient for velocitiy difference between set of wagons and outdoor air (in m/s)
end # mutable struct Train
Train()=Train("", 0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


## path separated in smaler sections
struct PathSection
    s_start::AbstractFloat      # starting point of the section (in m)
    s_startNext::AbstractFloat  # starting point of the next section (in m)
    v_limit::AbstractFloat      # paths speed limt (in m/s)
    n::AbstractFloat            # gradient (in ‰)
    # TODO: curve radius
    # TODO: turnout
    # TODO: tunnel
    f_Rp::AbstractFloat         # specific path resistance of the section (in ‰)
end # struct pathSection
PathSection()=(0.0, 0.0, 0.0, 0.0, 0.0)

struct Path
    name::String                    # paths name
    id                              # paths identifier
    sections::Vector{PathSection}   # list of PathSection elements
                                        # s_start in firt entry defines the paths beginning
                                        # s_startNext in last entry defines the paths ending
end # struct Path
Path()=("", 0, Vector{PathSection}())


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
    W_T::AbstractFloat      # mechanical work (in Ws)
    ΔW_T::AbstractFloat     # mechanical work in this step (in Ws)
    E::AbstractFloat        # energy consumption (in Ws)
    ΔE::AbstractFloat       # energy consumption in this step (in Ws)
    F_T::AbstractFloat      # tractive effort (in N)
    F_R::AbstractFloat      # resisting force (in N)
    F_Rp::AbstractFloat     # line resistanc (in N)
    F_Runion::AbstractFloat # vehicle resistance (in N)
    F_Rt::AbstractFloat     # traction unit resistance (in N)
    F_Rw::AbstractFloat     # set of wagons resistance (in N)
end # mutable struct DataPoint
DataPoint()=DataPoint(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# tried to insert copy on 15.07.2021 copy(original::DataPoint)=DataPoint(original.i, original.s, original.Δs, original.t, original.Δt, original.v, original.Δv, original.a, original.W_T, original.ΔW_T, original.E, original.ΔE, original.F_T, original.F_R, original.F_Rp, original.F_Runion, original.F_Rt, original.F_Rw)
DataPoint(original::DataPoint)=DataPoint(original.i, original.s, original.Δs, original.t, original.Δt, original.v, original.Δv, original.a, original.W_T, original.ΔW_T, original.E, original.ΔE, original.F_T, original.F_R, original.F_Rp, original.F_Runion, original.F_Rt, original.F_Rw)



## different sections the whole path can be devided in the following
## smallest section of the path is the behavior section. It relates to the containing data points via their identifier.
mutable struct BehaviorSection
    type::String                # type of behavior section: "starting", "cruisingBeforeAcceleration", "acceleration", "cruising", "coasting", "cruisingAfterCoasting" or "braking"
    s_total::AbstractFloat      # total length  (in m)
    s_start::AbstractFloat      # first position (in m)
    s_end::AbstractFloat        # last position  (in m)
    t_total::AbstractFloat      # total running time (in s)
    E_total::AbstractFloat      # total energy consumption (in Ws)
    v_entry::AbstractFloat      # entry speed (in m/s)
    v_exit::AbstractFloat       # exit speed (in m/s)
    dataPoints::Vector{Integer}  # list of identifiers of the containing data points
end # mutable struct BehaviorSection
BehaviorSection()=BehaviorSection("", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [])
function BehaviorSection(original::BehaviorSection)
    bsDataPoints=[]
    for i in 1:length(original.dataPoints)
        push!(bsDataPoints, original.dataPoints[i])
    end
    return BehaviorSection(original.type, original.s_total, original.s_start, original.s_end, original.t_total, original.E_total, original.v_entry, original.v_exit, bsDataPoints)
end

## a characteristic section is a part of the moving section. It contains behavior sections.
mutable struct CharacteristicSection
    id::Integer                 # identifier
    s_total::AbstractFloat      # total length  (in m)
    s_start::AbstractFloat      # first position (in m)
    s_end::AbstractFloat        # last position  (in m)
    t_total::AbstractFloat      # total running time (in s)
    E_total::AbstractFloat      # total energy consumption (in Ws)
    v_limit::AbstractFloat      # speed limit (in m/s)
    v_reach::AbstractFloat      # maximum reachable speed (in m/s)
    v_entry::AbstractFloat      # maximum entry speed (in m/s)
    v_exit::AbstractFloat       # maximum exit speed (in m/s)
    f_Rp::AbstractFloat         # spedific line resistance (in ‰)
    behaviorSections::AbstractDict{String, BehaviorSection}   # list of containing behavior sections
end # mutable struct CharacteristicSection
CharacteristicSection()=CharacteristicSection(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Dict{String, BehaviorSection}())
function CharacteristicSection(original::CharacteristicSection)
    copy=CharacteristicSection(original.id, original.s_total, original.s_start, original.s_end, original.t_total, original.E_total, original.v_limit, original.v_reach, original.v_entry, original.v_exit, original.f_Rp, Dict{String, BehaviorSection}())
    allBs=["starting", "cruisingBeforeAcceleration","acceleration", "cruising", "coasting","cruisingAfterCoasting", "braking"]
    for bs in 1: length(allBs)
        if haskey(original.behaviorSections, allBs[bs])
            merge!(copy.behaviorSections, Dict(allBs[bs]=>BehaviorSection(get(original.behaviorSections, allBs[bs], BehaviorSection()))))
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
    drivingCourseModified::Vector{DataPoint}     #drivingCourse for the modified characteristic section
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


## a moving section contains all the smaller sections from one stop to an other
mutable struct MovingSection
    id                                  # identifier
    s_total::AbstractFloat              # total length (in m)
    s_start::AbstractFloat              # first position (in m)
    s_end::AbstractFloat                # last position (in m)
    t_total::AbstractFloat              # total running time (in s)
    E_total::AbstractFloat              # total energy consumption (in Ws)
    t_recovery::AbstractFloat           # total recovery time for energy-saving modifications (in s)
    t_recoveryAvailable::AbstractFloat  # still available recovery time for energy-saving modifications (in s)
    characteristicSections::Vector{CharacteristicSection}   # list of containing characteristic sections
    energySavingModifications::Vector{EnergySavingModification}   # list of containing all the used energy saving modifications
end # mutable struct MovingSection
MovingSection()=MovingSection(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [], [])

function MovingSection(original::MovingSection)
    copy=MovingSection(original.id, original.s_total, original.s_start, original.s_end, original.t_total, original.E_total, original.t_recovery, original.t_recoveryAvailable, [], [])
    for csId in 1:length(original.characteristicSections)
        push!(copy.characteristicSections, CharacteristicSection(original.characteristicSections[csId]))
    end #for
    for modId in 1:length(original.energySavingModifications)
        push!(copy.energySavingModifications, EnergySavingModification(original.energySavingModifications[modId]))
    end #for
    return copy
end #function CharacteristicSection

end #module
