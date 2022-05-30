#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Martin Scheidt, Max Kannenberg"
# __copyright__     = "2022"
# __license__       = "ISC"

struct Settings

    massModel::Symbol    # model type of train mass ":mass_point" or ":homogeneous_strip".
    stepVariable::Symbol # variable of the linear multistep method: ":distance", ":time" or ":velocity".
    stepSize::Real       # step size, unit depends on stepVariable - :distance in meter, time in seconds and velocity in meter/second.
    approxLevel::Int     # value for approximation; used when rounding or iterating.
    outputDetail::Symbol # single Float() ":running_time", Array() of ":points_of_interest",
                         # complete Array() ":driving_course", or Dict() ":everything".
    outputFormat::Symbol # output as ":dataframe" or as ":dict".

end #struct Settings

struct Path

    name::String     # a name or description of the path
    id::String       # a short string as identifier
    uuid::UUID       # a unique identifier
    poi::Vector      # a vector of triples with points along the path
    sections::Vector # a vector of the characteristic sections

end #struct Path

struct Train

    name::String            # a name or description of the train
    id::String              # a short string as identifier
    uuid::UUID              # a unique identifier
    length::Real            # train length in meter
    m_train_full::Real      # mass of the full loaded train in kilogram
    m_td::Real              # mass on driving axles of the traction unit in kilogram
    m_tc::Real              # mass on the traction unit's carrying axles in kilogram
    m_w::Real               # mass of the set of wagons/cars/consist in kilogram
    ξ_train::Real           # rotation mass factor
    ξ_loco::Real            # rotation mass factor
    ξ_cars::Real            # rotation mass factor
    transportType::Symbol   # ":freight" or ":passenger" for resistance calculation
    v_limit::Real           # in m/s
    a_braking::Real         # in m/s^2

    # coefficients for the vehicle resistance
    # for the traction unit (F_Rt=f_Rtd0*m_td*g+f_Rtc0*m_tc*g+f_Rt2*m_loco*g*((v+Δv_air)/v00)^2)
    f_Rtd0::Real # coefficient for basic resistance due to the traction units driving axles (in ‰)
    f_Rtc0::Real # coefficient for basic resistance due to the traction units carring axles (in ‰)
    f_Rt2::Real  # coefficient for air resistance of the traction units (in ‰)

    # for the consist (set of wagons) (F_Rw=m_w*g*(f_Rw0+f_Rw1*v/v00+f_Rw2*((v+Δv_air)/v00)^2))
    f_Rw0::Real # coefficient for the consists basic resistance (in ‰)
    f_Rw1::Real # coefficient for the consists resistance to rolling (in ‰)
    f_Rw2::Real # coefficient fo the consistsr air resistance (in ‰)

    # tractive effort as pairs of speed and tractive effort
    tractiveEffort::Vector{Tuple{Real, Real}}   # [v in m/s, F_T in N]

end #struct Train
