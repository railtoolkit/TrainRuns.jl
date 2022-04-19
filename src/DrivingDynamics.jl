#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2022"
# __license__       = "ISC"

module DrivingDynamics

#########################
## literature the driving dynamics equations are based on:
##
## @incollection{Bruenger:2014, % Chapter 4
##   author    = {Brünger, Olaf and Dahlhaus, Elias},
##   year      = {2014},
##   title     = {Running Time Estimation},
##   pages     = {65--90},
##   booktitle = {Railway Timetabling \& Operations.},
##   editora   = {Hansen, Ingo A.},
##   editorb   = {Pachl, Jörn},
##   isbn      = {978-3-777-10462-1},
##   publisher = {Eurailpress DVV Media Group},
## }
## @article{Jaekel:2014,
##   author    = {Jaekel, Birgit and Albrecht, Thomas},
##   year      = {2014},
##   title     = {Comparative analysis of algorithms and models for train running simulation},
##   journal   = {Journal of Rail Transport Planning \& Management},
##   doi       = {10.1016/j.jrtpm.2014.06.002},
##   volume    = {4},
##   number    = {1-2},
##   pages     = {14--27},
##   publisher = {Elsevier},
## }
## @Book{Wende:2003,
##   author    = {Wende, Dietrich},
##   date      = {2003},
##   title     = {Fahrdynamik des Schienenverkehrs},
##   isbn      = {978-3-322-82961-0},
##   publisher = {Springer-Verlag},
## }
#########################

# export resisting forces and acceleration
export calcTractionUnitResistance, calcWagonsResistance, calcForceFromCoefficient, calcAcceleration,

# export step sizes in different units
calc_Δs_with_Δt, calc_Δs_with_Δv,
calc_Δt_with_Δs, calc_Δt_with_Δv, calc_Δt_with_constant_v,
calc_Δv_with_Δs, calc_Δv_with_Δt,
calc_ΔW, calc_ΔE,

# export braking information
calcBrakingDistance, calcBrakingStartVelocity, calcBrakingAcceleration

v00 = 100/3.6     # velocity factor (in m/s)
g = 9.81          # acceleration due to gravity (in m/s^2)            # TODO: should more digits of g be used?  g=9,80665 m/s^2

approximationLevel = 6  # value for approximation to intersections TODO further explanation (e.g. approximationLevel = 3 -> with stepSize 10 m the approximation will be calculated accurate on 10 mm ; 1s -> 1 ms; 1 km/h -> 3.6 mm/s)
# TODO: necessary here?


## calculate forces

#TODO: replace the ? ? ?
"""
    calcTractionUnitResistance(v, train)

Calculate the vehicle resistance for the traction unit of the `train` dependend on the velocity `v`.

...
# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `train::Dict`: ? ? ?
...

# Examples
```julia-repl
julia> calcTractionUnitResistance(30.0, ? ? ?)
? ? ?
```
"""
function calcTractionUnitResistance(v::AbstractFloat, train::Dict)
    # equation is based on [Wende:2003, page 151]
    f_Rtd0 = train[:f_Rtd0]         # coefficient for basic resistance due to the traction units driving axles (in ‰)
    f_Rtc0 = train[:f_Rtc0]         # coefficient for basic resistance due to the traction units carring axles (in ‰)
    F_Rt2 = train[:F_Rt2]           # coefficient for air resistance of the traction units (in N)
    m_td = train[:m_td]             # mass on the traction unit's driving axles (in kg)
    m_tc = train[:m_tc]             # mass on the traction unit's carrying axles (in kg)
    Δv_t = train[:Δv_t]             # coefficient for velocitiy difference between traction unit and outdoor air (in m/s)

    F_R_tractionUnit = f_Rtd0/1000 * m_td * g + f_Rtc0/1000 * m_tc * g + F_Rt2 * ((v + Δv_t) /v00)^2   # vehicle resistance of the traction unit (in N)   # /1000 because of the unit ‰
    # TODO: use calcForceFromCoefficient? F_R_tractionUnit = calcForceFromCoefficient(f_Rtd0, m_td) + calcForceFromCoefficient(f_Rtc0, m_tc) + F_Rt2 * ((v + Δv_t) /v00)^2       # vehicle resistance of the traction unit (in N)
    return F_R_tractionUnit
    #TODO: same variable name like in the rest of the tool? return R_traction
    #TODO: just one line? return train[:f_Rtd0]/1000*train[:m_td]*g+train[:f_Rtc0]/1000*train[:m_tc]*g+train[:F_Rt2]*((v+train[:Δv_t])/v00)^2    # /1000 because of the unit ‰
end #function calcTractionUnitResistance

"""
TODO
calculate and return the wagons vehicle resistance dependend on the velocity
"""
function calcWagonsResistance(v::AbstractFloat, train::Dict)
    # equation is based on a combination of the equations of Strahl and Sauthoff [Wende:2003, page 153] with more detailled factors (Lehmann, page 135)
    f_Rw0 = train[:f_Rw0]       # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    f_Rw1 = train[:f_Rw1]       # coefficient for the consists resistance to rolling (in ‰)
    f_Rw2 = train[:f_Rw2]       # coefficient fo the consistsr air resistance (in ‰)
    m_w = train[:m_w]           # mass of the set of wagons (consist)  (in kg)
    Δv_w = train[:Δv_w]         # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)

    F_R_wagons = m_w *g *(f_Rw0/1000 + f_Rw1/1000 *v /v00 + f_Rw2/1000 * ((v + Δv_w) /v00)^2)     # vehicle resistance of the wagons (in N)      # /1000 because of the unit ‰
# TODO: use calcForceFromCoefficient?    F_R_wagons = calcForceFromCoefficient(f_Rw0, m_w) + calcForceFromCoefficient(f_Rw1, m_w) *v /v00 + calcForceFromCoefficient(f_Rw2, m_w) * ((v + Δv_w) /v00)^2     # vehicle resistance of the wagons (in N)
    return F_R_wagons
end #function calcWagonsResistance

function calcForceFromCoefficient(f_R::Real, m::Real)
    # equation is based on [Wende:2003, page 8]

    # f_R: specific resistance (in ‰)
    # m: vehicle's mass (in kg)

    F_R = f_R /1000 *m *g     # Resisting Force (in N)  # /1000 because of the unit ‰
    return F_R
end #function calcForceFromCoefficient

function calcAcceleration(F_T::Real, F_R::Real, m_train::Real, ξ_train::Real)
    # equation is based on [Bruenger:2014, page 72] with a=dv/dt

    # F_T: tractive effort (in N)
    # F_R: resisting forces (in N)
    # m_train: train's mass (in kg)
    # ξ_train: train's rotation mass factor (without unit)

    a = (F_T - F_R) /m_train /ξ_train      # acceleration (in m/s)
    return a
end #function calcAcceleration

function calc_Δs_with_Δt(Δt::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    # Δt: time step (in s)
    # a_prev: acceleration from previous data point
    # v_prev: velocitiy from previous data point
    Δs = Δt * (2*v_prev + Δt*a_prev) /2        # step size (in m)
    return Δs
end #function calc_Δs_with_Δt

function calc_Δs_with_Δv(Δv::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    # Δv: velocity step (in m/s)
    # a_prev: acceleration from previous data point
    # v_prev: velocitiy from previous data point
    Δs = ((v_prev + Δv)^2 - v_prev^2)/2/a_prev      # step size (in m)
    return Δs
end #function calc_Δs_with_Δv

function calc_Δt_with_Δs(Δs::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    # Δs: distance step (in m)
    # a_prev: acceleration from previous data point
    # v_prev: velocitiy from previous data point

    Δt = sign(a_prev) *sqrt((v_prev /a_prev)^2 + 2 *Δs /a_prev) - v_prev /a_prev       # step size (in m/s)
    return Δt
end #function calc_Δt_with_Δs

function calc_Δt_with_Δv(Δv::Real, a_prev::Real)
    # equation is based on [Wende:2003, page 37]

    # Δv: velocity step (in m/s)
    # a_prev: acceleration from previous data point
    Δt = Δv /a_prev        # step size (in s)
    return Δt
end #function calc_Δt_with_Δv

function calc_Δt_with_constant_v(Δs::Real, v::Real)
    # equation is based on [Wende:2003, page 37]

    # Δs: distance step (in m)
    # v: constant velocity (in m/s)
    Δt = Δs /v        # step size (in s)
    return Δt
end #function calc_Δt_with_constant_v

function calc_Δv_with_Δs(Δs::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    # Δs: distance step (in m)
    # a_prev: acceleration from previous data point
    # v_prev: velocitiy from previous data point
    Δv = sqrt(v_prev^2 + 2*Δs*a_prev) - v_prev      # step size (in m/s)
    return Δv
end #function calc_Δv_with_Δs

function calc_Δv_with_Δt(Δt::Real, a_prev::Real)
    # equation is based on [Wende:2003, page 37]

    # Δt: time step (in s)
    # a_prev: acceleration from previous data point
    Δv = Δt * a_prev        # step size (in m/s)
    return Δv
end #function calc_Δv_with_Δt

function calc_ΔW(F_T_prev::Real, Δs::Real)
    # equation is based on [Wende:2003, page 17]

    # F_T_prev: tractive force from previous data point
    # Δs: distance step
    ΔW = F_T_prev * Δs      # mechanical work in this step (in Ws)
    return ΔW
end #function calc_ΔW

function calc_ΔE(ΔW::Real)
    # simplified equation is based on [Jaekel:2014, page 6]

    # ΔW: mechanical work in this step (in Ws)
    ΔE = ΔW                 # energy consumption in this step (in Ws)
    return ΔE
end #function calc_ΔW

function calcBrakingDistance(v_start::Real, v_end::Real, a_braking::Real)
    # equation is based on [Wende:2003, page 37]

    # v_start: velocity at the start of braking (in m/s)
    # v_end: target velocity at the end of braking (in m/s)
    # a_braking: constant braking acceleration (in m/s^2)
    s_braking = (v_end^2 - v_start^2) /2 /a_braking             # braking distance (in m)
    # TODO: also possible: calc_Δs_with_Δv(v_end-v_start, a_braking, v_start)
#    return max(0.0, ceil(s_braking, digits=approximationLevel))         # ceil is used to be sure that the train stops at s_exit in spite of rounding errors
    return max(0.0, ceil(s_braking, digits=approximationLevel +1))         # ceil is used to be sure that the train stops at s_exit in spite of rounding errors
end #function calcBrakingDistance

function calcBrakingStartVelocity(v_end::Real, a_braking::Real, s_braking::Real)
    # equation is based on [Wende:2003, page 37]

    # v_end: target velocity at the end of braking (in m/s)
    # a_braking: constant braking acceleration (in m/s^2)
    # s_braking: braking distance (in Ws)
    v_start = sqrt(v_end^2 - 2*a_braking *s_braking)          # braking start velocity (in m/s)
#    return floor(v_start, digits=approximationLevel)
    return floor(v_start, digits=approximationLevel +1)
end #function calcBrakingStartVelocity

function calcBrakingAcceleration(v_start::Real, v_end::Real, s_braking::Real)
    # equation is based on [Wende:2003, page 37]

    # v_start: braking start velocity (in m/s)
    # v_end: target velocity at the end of braking (in m/s)
    # s_braking: braking distance (in Ws)
    a_braking = (v_end^2 - v_start^2) /2 /s_braking       # constant braking acceleration (in m/s^2)
    return a_braking
end #function calcBrakingAcceleration

end #module DrivingDynamics
