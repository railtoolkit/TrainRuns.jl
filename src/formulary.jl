#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2022"
# __license__       = "ISC"

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
## @Book{Wende:2003,
##   author    = {Wende, Dietrich},
##   date      = {2003},
##   title     = {Fahrdynamik des Schienenverkehrs},
##   isbn      = {978-3-322-82961-0},
##   publisher = {Springer-Verlag},
## }
#########################

v00 = 100/3.6     # velocity factor (in m/s)

## calculate forces:

"""
    tractionUnitResistance(v, train)

Calculate the vehicle resistance in N for the traction unit of the `train` dependend on the velocity `v`.

# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `train::Train`: the struct defined in types.jl containing attributes with technical data.

# Examples
```julia-repl
julia> tractionUnitResistance(15.0, freight_train)
5461.127252
```
"""
function tractionUnitResistance(v::AbstractFloat, train::Train)
    # equation is based on [Wende:2003, page 151]
    f_Rtd0 = train.f_Rtd0 # coefficient for basic resistance due to the traction units driving axles (in ‰)
    f_Rtc0 = train.f_Rtc0 # coefficient for basic resistance due to the traction units carring axles (in ‰)
    f_Rt2  = train.f_Rt2  # coefficient for air resistance of the traction unit (in ‰)
    m_td   = train.m_td   # mass on the traction unit's driving axles (in kg)
    m_tc   = train.m_tc   # mass on the traction unit's carrying axles (in kg)

    F_R_tractionUnit = f_Rtd0/1000 * m_td * g + f_Rtc0/1000 * m_tc * g + f_Rt2/1000 * (m_td+m_tc) * g * ((v + Δv_air) /v00)^2   # vehicle resistance of the traction unit (in N)   # /1000 because of the unit ‰

    return F_R_tractionUnit
end #function tractionUnitResistance


"""
    freightWagonsResistance(v, train)

Calculate the vehicle resistance in N for the freight wagons of the `train` dependend on the velocity `v`.

# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `train::Train`: the struct defined in types.jl containing attributes with technical data.

# Examples
```julia-repl
julia> freightWagonsResistance(15.0, freight_train)
20900.732702639998
```
"""
function freightWagonsResistance(v::AbstractFloat, train::Train)
    # equation is based on the equation of Strahl [Wende:2003, page 153]
    f_Rw0  = train.f_Rw0  # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    f_Rw2  = train.f_Rw2  # coefficient fo the consistsr air resistance (in ‰)
    m_w    = train.m_w    # mass of the set of wagons (consist)  (in kg)

    F_R_wagons = m_w *g *(f_Rw0/1000 + f_Rw2/1000 * (v /v00)^2)     # vehicle resistance of freight wagons (in N) with Strahl      # /1000 because of the unit ‰
    return F_R_wagons
end #function calcWagonsResistance


"""
    passengerWagonsResistance(v, train)

Calculate the vehicle resistance in N for the passenger wagons of the `train` dependend on the velocity `v`.

# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `train::Train`: the struct defined in types.jl containing attributes with technical data.

# Examples
```julia-repl
julia> passengerWagonsResistance(15.0, longdistance_passenger_train)
14461.2708244928
```
"""
function passengerWagonsResistance(v::AbstractFloat, train::Train)
    # equation is based on the equations of Sauthoff [Wende:2003, page 153]
    f_Rw0  = train.f_Rw0  # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    f_Rw1  = train.f_Rw1  # coefficient for the consists resistance to rolling (in ‰)
    f_Rw2  = train.f_Rw2  # coefficient fo the consistsr air resistance (in ‰)
    m_w    = train.m_w    # mass of the set of wagons (consist)  (in kg)

    F_R_wagons = m_w *g *(f_Rw0/1000 + f_Rw1/1000 *v /v00 + f_Rw2/1000 * ((v + Δv_air) /v00)^2)     # vehicle resistance of passenger wagons (in N) with Sauthoff      # /1000 because of the unit ‰

    return F_R_wagons
end #function calcWagonsResistance

"""
    forceFromCoefficient(f, m)

Calculate the force in N with the coefficient `f` in ‰ and the vehicle's mass `m` in kg.

# Examples
```julia-repl
julia> forceFromCoefficient(1.5, 68000.0)
1000.2783
```
"""
function forceFromCoefficient(f::Real, m::Real)
    # equation is based on [Wende:2003, page 8]

    F = f /1000 *m *g     # force (in N)  # /1000 because of the unit ‰
    return F
end #function forceFromCoefficient



## calculate acceleration:

"""
    acceleration(F_T, F_R, m_train, ξ_train)

Calculate the acceleration in m/s^2 with train characteristics and forces.

# Arguments
- `F_T::Real`: the tractive effort in N.
- `F_R::Real`: the resisting forces in N.
- `m_train::Real`: the train's mass in kg.
- `ξ_train::Real`: the train's rotation mass factor (without unit).

# Examples
```julia-repl
julia> acceleration(94400.0, 1700.0, 88000.0, 1.08)
0.9753787878787878
```
"""
function acceleration(F_T::Real, F_R::Real, m_train::Real, ξ_train::Real)
    # equation is based on [Bruenger:2014, page 72] with a=dv/dt

    a = (F_T - F_R) /m_train /ξ_train      # acceleration (in m/s^2)
    return a
end #function acceleration


## calculate step sizes:

"""
    Δs_with_Δt(Δt, a_prev, v_prev)

Calculate the distance step in m with the time step in s, acceleration in m/s^2 and velocity in m/s.

See also [`Δs_with_Δv`](@ref), [`Δt_with_Δs`](@ref), [`Δt_with_Δv`](@ref), [`Δt_with_constant_v`](@ref), [`Δv_with_Δs`](@ref), [`Δv_with_Δt`](@ref).

# Examples
```julia-repl
julia> Δs_with_Δt(3.0, 0.5, 25.0)
77.25
```
"""
function Δs_with_Δt(Δt::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    Δs = Δt * (2*v_prev + Δt*a_prev) /2        # step size (in m)
    return Δs
end #function Δs_with_Δt


"""
    Δs_with_Δv(Δv, a_prev, v_prev)

Calculate the distance step in m with the velocity step in m/s, acceleration in m/s^2 and velocity in m/s.

See also [`Δs_with_Δt`](@ref), [`Δt_with_Δs`](@ref), [`Δt_with_Δv`](@ref), [`Δt_with_constant_v`](@ref), [`Δv_with_Δs`](@ref), [`Δv_with_Δt`](@ref).

# Examples
```julia-repl
julia> Δs_with_Δv(1.0, 0.5, 25.0)
51.0
```
"""
function Δs_with_Δv(Δv::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    Δs = ((v_prev + Δv)^2 - v_prev^2) /2 /a_prev      # step size (in m)
    return Δs
end #function Δs_with_Δv


"""
    Δt_with_Δs(Δs, a_prev, v_prev)

Calculate the time step in s with the distance step in m, acceleration in m/s^2 and velocity in m/s.

See also [`Δs_with_Δt`](@ref), [`Δs_with_Δv`](@ref), [`Δt_with_Δv`](@ref), [`Δt_with_constant_v`](@ref), [`Δv_with_Δs`](@ref), [`Δv_with_Δt`](@ref).

# Examples
```julia-repl
julia> Δt_with_Δs(10.0, 0.5, 25.0)
0.39841267341660824
```
"""
function Δt_with_Δs(Δs::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    Δt = sign(a_prev) *sqrt((v_prev /a_prev)^2 + 2 *Δs /a_prev) - v_prev /a_prev       # step size (in s)
    return Δt
end #function Δt_with_Δs


"""
    Δt_with_Δv(Δv, a_prev)

Calculate the time step in s with the velocity step in m/s and the acceleration in m/s^2.

See also [`Δs_with_Δt`](@ref), [`Δs_with_Δv`](@ref), [`Δt_with_Δs`](@ref), [`Δt_with_constant_v`](@ref), [`Δv_with_Δs`](@ref), [`Δv_with_Δt`](@ref).

# Examples
```julia-repl
julia> Δt_with_Δv(1.0, 0.5)
2.0
```
"""
function Δt_with_Δv(Δv::Real, a_prev::Real)
    # equation is based on [Wende:2003, page 37]

    Δt = Δv /a_prev        # step size (in s)
    return Δt
end #function Δt_with_Δv


"""
    Δt_with_constant_v(Δs, v)

Calculate the time step in s with the distance step in m and constant velocity in m/s.

See also [`Δs_with_Δt`](@ref), [`Δs_with_Δv`](@ref), [`Δt_with_Δs`](@ref), [`Δt_with_Δv`](@ref), [`Δv_with_Δs`](@ref), [`Δv_with_Δt`](@ref).

# Examples
```julia-repl
julia> Δt_with_constant_v(10.0, 25.0)
0.4
```
"""
function Δt_with_constant_v(Δs::Real, v::Real)
    # equation is based on [Wende:2003, page 37]

    Δt = Δs /v        # step size (in s)
    return Δt
end #function Δt_with_constant_v


"""
    Δv_with_Δs(Δs, a_prev, v_prev)

Calculate the velocity step in m/s with the distance step in m, acceleration in m/s^2 and velocity in m/s.

See also [`Δs_with_Δt`](@ref), [`Δs_with_Δv`](@ref), [`Δt_with_Δs`](@ref), [`Δt_with_Δv`](@ref), [`Δt_with_constant_v`](@ref), [`Δv_with_Δt`](@ref).

# Examples
```julia-repl
julia> Δv_with_Δs(10.0, 0.5, 25.0)
0.19920633670830412
```
"""
function Δv_with_Δs(Δs::Real, a_prev::Real, v_prev::Real)
    # equation is based on [Wende:2003, page 37]

    Δv = sqrt(v_prev^2 + 2*Δs*a_prev) - v_prev      # step size (in m/s)
    return Δv
end #function Δv_with_Δs


"""
    Δv_with_Δt(Δt, a_prev, v_prev)

Calculate the velocity step in m/s with the time step in s and acceleration in m/s^2.

See also [`Δs_with_Δt`](@ref), [`Δs_with_Δv`](@ref), [`Δt_with_Δs`](@ref), [`Δt_with_Δv`](@ref), [`Δt_with_constant_v`](@ref), [`Δv_with_Δs`](@ref).

# Examples
```julia-repl
julia> Δv_with_Δt(3.0, 0.5)
1.5
```
"""
function Δv_with_Δt(Δt::Real, a_prev::Real)
    # equation is based on [Wende:2003, page 37]

    Δv = Δt * a_prev        # step size (in m/s)
    return Δv
end #function Δv_with_Δt



## calculate values for braking

"""
    brakingDistance(v_start, v_end, a_braking, approxLevel)

Calculate the braking distance in m with velocities in m/s and acceleration in m/s^2.

See also [`brakingAcceleration`](@ref), [`brakingStartVelocity`](@ref).

# Arguments
- `v_start::Real`: the velocity at the start of braking in m/s.
- `v_end::Real`: the target velocity at the end of braking in m/s.
- `a_braking::Real`: the constant braking acceleration in m/s^2.
- `approxLevel::Integer`: the last position behind the decimal point that is not rounded

# Examples
```julia-repl
julia> brakingDistance(25.0, 15.0, -0.4253, 3)
470.2563
```
"""
function brakingDistance(v_start::Real, v_end::Real, a_braking::Real, approxLevel::Integer)
    # equation is based on [Wende:2003, page 37]

    s_braking = (v_end^2 - v_start^2) /2 /a_braking             # braking distance (in m)
    return max(0.0, ceil(s_braking, digits= approxLevel +1))         # ceil is used to be sure that the train stops at s_exit in spite of rounding errors
end #function brakingDistance

"""
    brakingStartVelocity(v_end, a_braking, s_braking, approxLevel)

Calculate the maximum velocity in m/s where the train can start to brake to reach `v_end`.

See also [`brakingAcceleration`](@ref), [`brakingDistance`](@ref).

# Arguments
- `v_end::Real`: the target velocity at the end of braking in m/s.
- `a_braking::Real`: the constant braking acceleration in m/s^2.
- `s_braking::Real`: the braking distance in m.
- `approxLevel::Integer`: the last position behind the decimal point that is not rounded

# Examples
```julia-repl
julia> brakingStartVelocity(15.0, -0.4253, 500, 3)
25.4656
```
"""
function brakingStartVelocity(v_end::Real, a_braking::Real, s_braking::Real, approxLevel::Integer)
    # equation is based on [Wende:2003, page 37]

    v_start = sqrt(v_end^2 - 2*a_braking *s_braking)          # braking start velocity (in m/s)
    return floor(v_start, digits= approxLevel +1)
end #function brakingStartVelocity


"""
    brakingAcceleration(v_start, v_end, s_braking)

Calculate the acceleration in m/s^2 to decelerate from `v_start` to `v_end` in m/s on `s_braking` in m.

See also [`brakingDistance`](@ref), [`brakingStartVelocity`](@ref).

# Examples
```julia-repl
julia> brakingAcceleration(25.0, 15.0, 500)
-0.4
```
"""
function brakingAcceleration(v_start::Real, v_end::Real, s_braking::Real)
    # equation is based on [Wende:2003, page 37]

    a_braking = (v_end^2 - v_start^2) /2 /s_braking       # constant braking acceleration (in m/s^2)
    return a_braking
end #function brakingAcceleration
