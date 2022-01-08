module types
# definition of all the additional types and their constructors
export DataPoint#, BehaviorSection
 export copyBehaviorSection # TODO is it still necessary if there is no more mutable struct? can just copy(original) be used?
#export trainType

#@enum trainType passenger=1 freight=2 motorCoachTrain=3
#@enum behavior breakFree=1 clearing=2 acceleration=3 cruising=4diminishing=6 coasting=7 cruisingAfterCoasting=8 braking=9 standstill=10
## a data point is the smallest element of the driving course. One step of the step approach is between two data points
mutable struct DataPoint
    i::Integer              # identifier and counter variable of the dricing course
    behavior::String        # type of behavior section the DataPoint ist part of ("breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "cruisingAfterCoasting", "braking" or "standstill")
                            # a data point which is the last point of one behavior section and the first point of the next behavior section will be attached to the latter
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
    F_T::AbstractFloat          # tractive effort (in N)
    F_R::AbstractFloat          # resisting force (in N)
    R_path::AbstractFloat       # path resistance (in N)
    R_train::AbstractFloat      # train resistance (in N)
    R_traction::AbstractFloat   # traction unit resistance (in N)
    R_wagons::AbstractFloat    # set of wagons resistance (in N)
end # mutable struct DataPoint
DataPoint()=DataPoint(0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# tried to insert copy on 15.07.2021 copy(original::DataPoint)=DataPoint(original.i, original.s, original.Δs, original.t, original.Δt, original.v, original.Δv, original.a, original.W, original.ΔW, original.E, original.ΔE, original.F_T, original.F_R, original.R_path, original.R_train, original.R_traction, original.R_wagons)
DataPoint(original::DataPoint)=DataPoint(original.i, original.behavior, original.s, original.Δs, original.t, original.Δt, original.v, original.Δv, original.a, original.W, original.ΔW, original.E, original.ΔE, original.F_T, original.F_R, original.R_path, original.R_train, original.R_traction, original.R_wagons)


## different sections the whole path can be devided in the following
## smallest section of the path is the behavior section. It relates to the containing data points via their identifier.
function copyBehaviorSection(original::Dict)
    bsDataPoints=[]
    for i in 1:length(original[:dataPoints])
        push!(bsDataPoints, original[:dataPoints][i])
    end
    copiedBS = Dict(#:type => behavior,                 # type of behavior section: breakFree, clearing, acceleration, cruising, diminishing, coasting, cruisingAfterCoasting, braking or standstill
                :type => original[:type],           # type of behavior section: "breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "cruisingAfterCoasting", "braking" or "standstill"
                :length => original[:length],       # total length  (in m)
                :s_entry => original[:s_entry],     # first position (in m)
                :s_exit => original[:s_exit],       # last position  (in m)
                :t => original[:t],                 # total running time (in s)
                :E => original[:E],                 # total energy consumption (in Ws)
                :v_entry => original[:v_entry],     # entry speed (in m/s)
                :v_exit => original[:v_exit],       # exit speed (in m/s)
                :dataPoints => bsDataPoints)        # list of identifiers of the containing data points
    return copiedBS
end

end #module
