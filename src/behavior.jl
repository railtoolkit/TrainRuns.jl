#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

"""
    addBreakFreeSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the breakFree section and add them to `drivingCourse`.

See also [`addClearingSection!`](@ref), [`addAcceleratingSection!`](@ref), [`addCruisingSection!`](@ref), [`addDiminishingSection!`](@ref), [`addCoastingSection!`](@ref), [`addBrakingSection!`](@ref), [`addHalt!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
```julia-repl
julia> addBreakFreeSection(drivingCourse_with_starting_point, stateFlags, CSs, 1, settings_default, train_longdistance)
(Dict[Dict{Symbol, Any}(:behavior => "breakFree", :F_T => 300000, :R_wagons => 7309.09433933, :R_path => 0.0, :s => 0, :v => 0.0, :R_train => 9505.53877308, :R_traction => 2196.44443375, :a => 0.6143175668391081, :t => 0.0, :F_R => 9505.53877308), Dict{Symbol, Any}(:behavior => "breakFree", :F_T => 300000.0, :R_wagons => 8848.180631278232, :R_path => 0.0, :s => 20, :v => 4.957086107136361, :R_train => 11471.657638238565, :R_traction => 2623.4770069603337, :a => 0.6101597548372565, :t => 8.069256643013498, :F_R => 11471.657638238565)], Dict{Symbol, Bool}(:previousSpeedLimitReached => 0, :speedLimitReached => 0, :brakingStartReached => 0, :resistingForceNegative => 0, :endOfCSReached => 0, :tractionDeficit => 0, :error => 0))
```
"""
function addBreakFreeSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    # INFO: currently the values of the breakFree section will be calculated like in the accelerating section
    CS = CSs[csId]

    # conditions for the break free section
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    trainIsHalting = drivingCourse[end][:v] == 0.0

    if trainIsHalting && !endOfCSReached
        drivingMode = "breakFree"
        drivingCourse[end][:behavior] = drivingMode
        startingPoint = length(drivingCourse)

        # traction effort and resisting forces (in N)
        calculateForces!(
            drivingCourse[end],
            CSs,
            csId,
            "accelerating",
            train,
            settings.massModel
        )    # currently the tractive effort is calculated like in the accelerating section

        # calculate the breakFree section with calculating the accelerating section and just using the first step and removing the rest
        try
            (drivingCourse, stateFlags) = addAcceleratingSection!(
                drivingCourse,
                stateFlags,
                CSs,
                csId,
                settings,
                train
            )
        catch
            (acceleratingError)
            @error "This error happened during the break free phase that is using the accelerating function:"
            rethrow(acceleratingError)
        end

        # delete every supportPoint except the first two
        while length(drivingCourse) > startingPoint + 1
            pop!(drivingCourse)
        end

        # change the accelerating data to break free
        drivingCourse[end - 1][:behavior] = drivingMode
        drivingCourse[end][:behavior] = drivingMode
    end # else: return the characteristic section without a breakFree section

    # determine state flags
    s_braking = brakingDistance(
        drivingCourse[end][:v],
        CS[:v_exit],
        train.a_braking,
        settings.approxLevel
    )

    stateFlags[:endOfCSReached] = drivingCourse[end][:s] >= CS[:s_exit]
    stateFlags[:brakingStartReached] = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
    stateFlags[:tractionDeficit] = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = false
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_limit]
    stateFlags[:error] = drivingCourse[end][:v] > CS[:v_limit] ||
                         drivingCourse[end][:s] > CS[:s_exit]

    return (drivingCourse, stateFlags)
end #function addBreakFreeSection!

"""
    addClearingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the clearing section and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addCruisingSection!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addClearingSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    CS = CSs[csId]

    if stateFlags[:previousSpeedLimitReached]
        lowestSpeedLimit = getLowestSpeedLimit(
            CSs, csId, drivingCourse[end][:s], train.length)
        s_braking = brakingDistance(
            drivingCourse[end][:v],
            CS[:v_exit],
            train.a_braking,
            settings.approxLevel
        )

        s_clearing = min(
            CS[:s_exit] - drivingCourse[end][:s] - s_braking,
            lowestSpeedLimit[:s_end] - drivingCourse[end][:s]
        )
        if s_clearing > 0.0
            (drivingCourse, stateFlags) = addCruisingSection!(
                drivingCourse,
                stateFlags,
                CSs,
                csId,
                settings,
                train,
                "clearing",
                s_clearing
            )
            calculateForces!(
                drivingCourse[end],
                CSs,
                csId,
                "accelerating",
                train,
                settings.massModel
            )
        else
            @error "clearing <=0.0 although it has to be >0.0 in CS $csId"
        end
        #stateFlags[:previousSpeedLimitReached] = false
        lowestSpeedLimit = getLowestSpeedLimit(
            CSs, csId, drivingCourse[end][:s], train.length)
        stateFlags[:previousSpeedLimitReached] = lowestSpeedLimit[:v] != CS[:v_limit] &&
                                                 drivingCourse[end][:v] >=
                                                 lowestSpeedLimit[:v]
    else
        stateFlags[:error] = true
    end

    return (drivingCourse, stateFlags)
end #function addClearingSection

"""
    addAcceleratingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the accelerating section and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addClearingSection!`](@ref), [`addCruisingSection!`](@ref), [`addDiminishingSection!`](@ref), [`addCoastingSection!`](@ref), [`addBrakingSection!`](@ref), [`addHalt!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addAcceleratingSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    CS = CSs[csId]

    calculateForces!(
        drivingCourse[end],
        CSs,
        csId,
        "accelerating",
        train,
        settings.massModel
    )
    s_braking = brakingDistance(
        drivingCourse[end][:v],
        CS[:v_exit],
        train.a_braking,
        settings.approxLevel
    )

    # conditions for the accelerating section
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] ||
                          stateFlags[:brakingStartReached]
    previousSpeedLimitReached = stateFlags[:previousSpeedLimitReached]
    speedLimitReached = drivingCourse[end][:v] >= CS[:v_limit] ||
                        stateFlags[:speedLimitReached]

    # use the conditions for the accelerating section
    if !speedLimitReached && !endOfCSReached && tractionSurplus && !brakingStartReached
        drivingMode = "accelerating"
        drivingCourse[end][:behavior] = drivingMode

        lowestSpeedLimit = getLowestSpeedLimit(
            CSs, csId, drivingCourse[end][:s], train.length)
        previousSpeedLimitReached = lowestSpeedLimit[:v] != CS[:v_limit] &&
                                    drivingCourse[end][:v] >= lowestSpeedLimit[:v]

        while !speedLimitReached &&
                  !endOfCSReached &&
                  tractionSurplus &&
                  !brakingStartReached &&
                  !previousSpeedLimitReached
            currentStepSize = settings.stepSize   # initialize the step size that can be reduced near intersections
            nextPoiPosition = getNextPoiPosition(
                CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPoiPosition

            for cycle in 1:(settings.approxLevel + 1)   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !speedLimitReached &&
                          !brakingStartReached &&
                          !pointOfInterestReached &&
                          tractionSurplus &&
                          !previousSpeedLimitReached
                    if drivingCourse[end][:s] >= lowestSpeedLimit[:s_end]
                        # could be asked after creating an support point. This way here prevents even a minimal exceedance of speed limit. On the other hand the train cruises possibly a little to long
                        lowestSpeedLimit = getLowestSpeedLimit(
                            CSs,
                            csId,
                            drivingCourse[end][:s],
                            train.length
                        )
                    end

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(
                        drivingCourse[end][:F_T],
                        drivingCourse[end][:F_R],
                        train.m_train_full,
                        train.ξ_train
                    )

                    # create the next support point
                    push!(
                        drivingCourse,
                        moveAStep(
                            drivingCourse[end],
                            settings.stepVariable,
                            currentStepSize,
                            csId
                        )
                    )
                    drivingCourse[end][:behavior] = drivingMode

                    calculateForces!(
                        drivingCourse[end],
                        CSs,
                        csId,
                        drivingMode,
                        train,
                        settings.massModel
                    )

                    # conditions for the next while cycle
                    s_braking = brakingDistance(
                        drivingCourse[end][:v],
                        CS[:v_exit],
                        train.a_braking,
                        settings.approxLevel
                    )

                    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
                    speedLimitReached = drivingCourse[end][:v] >= CS[:v_limit]
                    previousSpeedLimitReached = lowestSpeedLimit[:v] < CS[:v_limit] &&
                                                (drivingCourse[end][:v] >
                                                 lowestSpeedLimit[:v] ||
                                                 (drivingCourse[end][:v] ==
                                                  lowestSpeedLimit[:v] &&
                                                  drivingCourse[end][:s] <
                                                  lowestSpeedLimit[:s_end]))
                    pointOfInterestReached = drivingCourse[end][:s] >= nextPoiPosition   # POIs include s_exit as well
                    tractionSurplus = drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                end #while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel + 1
                    if drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: F_T=",
                        drivingCourse[end][:F_T],
                        " <= F_R=",
                        drivingCourse[end][:F_R]
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif s_braking > 0.0 &&
                           drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        ",+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " > s_exit=", CS[:s_exit]
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " > nextPOI=",
                        nextPoiPosition
                        if settings.stepVariable == :distance
                            currentStepSize = nextPoiPosition -
                                              drivingCourse[end - 1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > lowestSpeedLimit[:v]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " > v_lowestLimit=",
                        lowestSpeedLimit[:v]
                        if settings.stepVariable == :velocity
                            currentStepSize = lowestSpeedLimit[:v] -
                                              drivingCourse[end - 1][:v]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        ",+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " == s_exit=", CS[:s_exit]
                        break

                    elseif drivingCourse[end][:v] == lowestSpeedLimit[:v]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " == v_lowestLimit=",
                        lowestSpeedLimit[:v]
                        break

                    elseif drivingCourse[end][:s] == nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " == nextPOI=",
                        nextPoiPosition
                        break

                    else
                        @info "v=",
                        drivingCourse[end][:v],
                        "   v_limit= ",
                        CS[:v_limit],
                        "  v_lowestLimit=",
                        lowestSpeedLimit[:v]
                        @info "s=",
                        drivingCourse[end][:s],
                        "   s_exit=",
                        CS[:s_exit],
                        "   s+s_braking=",
                        drivingCourse[end][:s] +s_braking,
                        "   nextPOI=",
                        nextPoiPosition
                        @info "F_T=",
                        drivingCourse[end][:F_T],
                        "   F_R=",
                        drivingCourse[end][:F_R]

                        @error "ERROR at accelerating section: With the step variable ",
                        settings.stepVariable,
                        " the while loop will be left although v<v_limit and s<s_exit in CS",
                        csId,
                        "  with s=",
                        drivingCourse[end][:s],
                        " m and v=",
                        drivingCourse[end][:v],
                        " m/s"
                    end
                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    brakingStartReached = false
                    previousSpeedLimitReached = false
                    speedLimitReached = false
                    pointOfInterestReached = false
                    tractionSurplus = true

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] > lowestSpeedLimit[:v]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " > v_lowestLimit=",
                        lowestSpeedLimit[:v],
                        "with v_limit=",
                        CS[:v_limit]

                        pop!(drivingCourse)

                        # conditions for the next section
                        brakingStartReached = false

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        ",+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " > s_exit=", CS[:s_exit]
                        if s_braking > 0.0
                            pop!(drivingCourse)

                        else
                            drivingCourse[end][:s] = CS[:s_exit] # round s down to CS[:s_exit]
                        end

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " > nextPoiPosition=",
                        nextPoiPosition
                        drivingCourse[end][:s] = nextPoiPosition # round s down to nextPoiPosition

                    elseif drivingCourse[end][:F_T] <= drivingCourse[end][:F_R]
                        @logmsg Trace "in CS",
                        csId,
                        " accelerating cycle",
                        cycle,
                        " case: F_T=",
                        drivingCourse[end][:F_T],
                        " <= F_R=",
                        drivingCourse[end][:F_R]

                    else
                        if drivingCourse[end][:s] + s_braking == CS[:s_exit]
                            @logmsg Trace "in CS",
                            csId,
                            " accelerating cycle",
                            cycle,
                            " else case and there: s +s_braking=",
                            drivingCourse[end][:s],
                            ",+",
                            s_braking,
                            " = ",
                            drivingCourse[end][:s] +s_braking, " > s_exit=", CS[:s_exit]

                        elseif drivingCourse[end][:v] == lowestSpeedLimit[:v]
                            @logmsg Trace "in CS",
                            csId,
                            " accelerating cycle",
                            cycle,
                            " case: v=",
                            drivingCourse[end][:v],
                            " == v_lowestLimit=",
                            lowestSpeedLimit[:v]
                        end
                    end
                end
            end #for

            if drivingCourse[end][:s] == CS[:s_exit]
                endOfCSReached = true
            end
        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = !(tractionSurplus ||
                                     drivingCourse[end][:F_T] == drivingCourse[end][:F_R])
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = previousSpeedLimitReached
    stateFlags[:speedLimitReached] = speedLimitReached
    stateFlags[:error] = !(
        endOfCSReached ||
        brakingStartReached ||
        stateFlags[:tractionDeficit] ||
        previousSpeedLimitReached ||
        speedLimitReached
    )

    return (drivingCourse, stateFlags)
end #function addAcceleratingSection!

"""
    addCruisingSection!(drivingCourse, stateFlags, CSs, csId, settings, train, cruisingType, s_cruising)

Calculate the support points of the cruising section and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addClearingSection!`](@ref), [`addAcceleratingSection!`](@ref), [`addDiminishingSection!`](@ref), [`addCoastingSection!`](@ref), [`addBrakingSection!`](@ref), [`addHalt!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.
- `cruisingType::String`: the type of crusing respectively if applicable the reason why the train has to cruise
- `s_cruising::Real`: the planned maximum crusing distance

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addCruisingSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train,
        cruisingType::String,
        s_cruising::Real
)
    CS = CSs[csId]

    trainIsClearing = cruisingType == "clearing"
    trainIsBrakingDownhill = cruisingType == "downhillBraking"

    # traction effort and resisting forces (in N)
    if !trainIsBrakingDownhill
        calculateForces!(
            drivingCourse[end],
            CSs,
            csId,
            "cruising",
            train,
            settings.massModel
        )
    else
        calculateForces!(
            drivingCourse[end],
            CSs,
            csId,
            "braking",
            train,
            settings.massModel
        )
    end

    s_braking = brakingDistance(
        drivingCourse[end][:v],
        CS[:v_exit],
        train.a_braking,
        settings.approxLevel
    )

    # conditions for cruising section
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] ||
                          stateFlags[:brakingStartReached]
    speedIsValid = drivingCourse[end][:v] > 0.0 && drivingCourse[end][:v] <= CS[:v_limit]
    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
    targetPositionReached = s_cruising == 0.0
    resistingForceNegative = drivingCourse[end][:F_R] < 0

    if speedIsValid && !brakingStartReached && !tractionDeficit && !targetPositionReached
        drivingMode = cruisingType
        drivingCourse[end][:behavior] = drivingMode
        targetPosition = min(drivingCourse[end][:s] + s_cruising, CS[:s_exit])

        # traction effort and resisting forces (in N)
        if !trainIsBrakingDownhill
            calculateForces!(
                drivingCourse[end],
                CSs,
                csId,
                "cruising",
                train,
                settings.massModel
            )
        else
            calculateForces!(
                drivingCourse[end],
                CSs,
                csId,
                "braking",
                train,
                settings.massModel
            )
        end

        if settings.massModel == :homogeneous_strip && csId > 1
            # conditions for cruising section
            trainInPreviousCS = drivingCourse[end][:s] < CS[:s_entry] + train.length
            targetPositionReached = drivingCourse[end][:s] >= targetPosition
            resistingForceNegative = drivingCourse[end][:F_R] < 0.0

            # use the conditions for the cruising section
            while trainInPreviousCS &&
                      !targetPositionReached &&
                      !tractionDeficit &&
                      (
                          trainIsClearing ||
                          (trainIsBrakingDownhill == resistingForceNegative)
                      ) # while clearing tractive or braking force can be used
                currentStepSize = settings.stepSize
                nextPoiPosition = getNextPoiPosition(
                    CS[:pointsOfInterest], drivingCourse[end][:s])
                pointOfInterestReached = drivingCourse[end][:s] >= nextPoiPosition

                for cycle in 1:(settings.approxLevel + 1)   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                    while trainInPreviousCS &&
                              !targetPositionReached &&
                              !pointOfInterestReached &&
                              !tractionDeficit &&
                              (
                                  trainIsClearing ||
                                  (trainIsBrakingDownhill == resistingForceNegative)
                              ) # while clearing tractive or braking force can be used
                        # tractive effort (in N):
                        if !trainIsBrakingDownhill
                            drivingCourse[end][:F_T] = min(
                                drivingCourse[end][:F_T],
                                max(0.0, drivingCourse[end][:F_R])
                            )
                        else
                            drivingCourse[end][:F_T] = 0.0
                        end

                        # acceleration (in m/s^2):
                        drivingCourse[end][:a] = 0.0

                        # create the next support point
                        if settings.stepVariable == :distance ||
                           settings.stepVariable == :time
                            push!(
                                drivingCourse,
                                moveAStep(
                                    drivingCourse[end],
                                    settings.stepVariable,
                                    currentStepSize,
                                    csId
                                )
                            )
                        else
                            push!(
                                drivingCourse,
                                moveAStep(
                                    drivingCourse[end],
                                    settings.stepVariable,
                                    train.length / (10.0^cycle),
                                    csId
                                )
                            )
                        end
                        drivingCourse[end][:behavior] = drivingMode

                        # traction effort and resisting forces (in N)
                        calculateForces!(
                            drivingCourse[end],
                            CSs,
                            csId,
                            "default",
                            train,
                            settings.massModel
                        )

                        # conditions for the next while cycle
                        pointOfInterestReached = drivingCourse[end][:s] >=
                                                 nextPoiPosition   # POIs include s_exit as well
                        tractionDeficit = drivingCourse[end][:F_T] <
                                          drivingCourse[end][:F_R]
                        targetPositionReached = drivingCourse[end][:s] >= targetPosition
                        trainInPreviousCS = drivingCourse[end][:s] <
                                            CS[:s_entry] + train.length
                        resistingForceNegative = drivingCourse[end][:F_R] < 0.0
                    end #while

                    # check which limit was reached and adjust the currentStepSize for the next cycle
                    if cycle < settings.approxLevel + 1
                        if drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                            currentStepSize = settings.stepSize / 10.0^cycle

                        elseif !trainIsBrakingDownhill && resistingForceNegative
                            currentStepSize = settings.stepSize / 10.0^cycle

                        elseif trainIsBrakingDownhill && !resistingForceNegative
                            currentStepSize = settings.stepSize / 10.0^cycle

                        elseif drivingCourse[end][:s] > nextPoiPosition
                            if settings.stepVariable == :distance
                                currentStepSize = nextPoiPosition -
                                                  drivingCourse[end - 1][:s]
                            else
                                currentStepSize = settings.stepSize / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] > targetPosition
                            if settings.stepVariable == :distance
                                currentStepSize = targetPosition -
                                                  drivingCourse[end - 1][:s]
                            else
                                currentStepSize = settings.stepSize / 10.0^cycle
                            end

                        elseif drivingCourse[end][:s] == targetPosition
                            break

                        elseif drivingCourse[end][:s] >= CS[:s_entry] + train.length
                            break

                        elseif drivingCourse[end][:s] == nextPoiPosition
                            break

                        elseif !trainInPreviousCS
                            break

                        else
                            error(
                                "ERROR at cruising section: With the step variable ",
                                settings.stepVariable,
                                " the while loop will be left although the if cases don't apply in CS",
                                csId,
                                "  with s=",
                                drivingCourse[end][:s],
                                " m and v=",
                                drivingCourse[end][:v],
                                " m/s"
                            )
                        end

                        # delete last support point for recalculating the last step with reduced step size
                        pop!(drivingCourse)

                        # conditions for the next for cycle
                        pointOfInterestReached = false
                        tractionDeficit = false
                        targetPositionReached = false
                        trainInPreviousCS = true
                        resistingForceNegative = drivingCourse[end][:F_R] < 0.0

                    else # if the level of approximation is reached
                        if drivingCourse[end][:s] > nextPoiPosition
                            drivingCourse[end][:s] = nextPoiPosition # round s down to nextPoiPosition
                        elseif drivingCourse[end][:s] > targetPosition
                            if drivingMode != "clearing"
                                pop!(drivingCourse)
                            end
                        elseif drivingCourse[end][:s] == targetPosition
                            break
                        elseif drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                            break
                        elseif !trainIsBrakingDownhill && resistingForceNegative
                            break
                        elseif trainIsBrakingDownhill && !resistingForceNegative
                            break
                        elseif !trainInPreviousCS
                            break

                        else
                        end
                    end
                end #for
            end #while
        end #if

        # conditions for the next while cycle
        targetPositionReached = drivingCourse[end][:s] >= targetPosition
        tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
        resistingForceNegative = drivingCourse[end][:F_R] < 0.0

        while !targetPositionReached && !tractionDeficit &&
                  (trainIsClearing || (trainIsBrakingDownhill == resistingForceNegative)) # while clearing tractive or braking force can be used
            nextPoiPosition = getNextPoiPosition(
                CS[:pointsOfInterest], drivingCourse[end][:s])
            if nextPoiPosition > targetPosition
                nextPoiPosition = targetPosition
            end

            # tractive effort (in N):
            if !trainIsBrakingDownhill
                drivingCourse[end][:F_T] = min(
                    drivingCourse[end][:F_T], max(0.0, drivingCourse[end][:F_R]))
            else
                drivingCourse[end][:F_T] = 0.0
            end

            drivingCourse[end][:a] = 0.0    # acceleration (in m/s^2)

            # calculate the remaining cruising way
            s_cruisingRemaining = min(
                nextPoiPosition - drivingCourse[end][:s],
                targetPosition - drivingCourse[end][:s]
            )

            # create the next support point
            push!(
                drivingCourse,
                moveAStep(drivingCourse[end], :distance, s_cruisingRemaining, csId)
            )
            drivingCourse[end][:behavior] = drivingMode

            calculateForces!(
                drivingCourse[end],
                CSs,
                csId,
                "default",
                train,
                settings.massModel
            )

            # conditions for the next while cycle
            targetPositionReached = drivingCourse[end][:s] >= targetPosition
            tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
            resistingForceNegative = drivingCourse[end][:F_R] < 0
        end #while
    end # else: return the characteristic section without a cruising section

    # set state flags
    stateFlags[:endOfCSReached] = drivingCourse[end][:s] == CS[:s_exit]
    s_braking = brakingDistance(
        drivingCourse[end][:v],
        CS[:v_exit],
        train.a_braking,
        settings.approxLevel
    )
    stateFlags[:brakingStartReached] = brakingStartReached ||
                                       drivingCourse[end][:s] + s_braking >= CS[:s_exit]
    stateFlags[:tractionDeficit] = tractionDeficit
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0.0
    lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
    stateFlags[:previousSpeedLimitReached] = lowestSpeedLimit[:v] != CS[:v_limit] &&
                                             drivingCourse[end][:v] >= lowestSpeedLimit[:v]
    stateFlags[:error] = !(
        targetPositionReached ||
        tractionDeficit ||
        !(
            cruisingType == "clearing" ||
            ((cruisingType == "downhillBraking") == resistingForceNegative)
        )
    )

    return (drivingCourse, stateFlags)
end #function addCruisingSection!

"""
    addDiminishingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the diminishing section (when using maximum tractive effort and still getting slower) and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addClearingSection!`](@ref), [`addAcceleratingSection!`](@ref), [`addCruisingSection!`](@ref), [`addCoastingSection!`](@ref), [`addBrakingSection!`](@ref), [`addHalt!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addDiminishingSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    CS = CSs[csId]

    calculateForces!(
        drivingCourse[end],
        CSs,
        csId,
        "diminishing",
        train,
        settings.massModel
    )

    s_braking = brakingDistance(
        drivingCourse[end][:v],
        CS[:v_exit],
        train.a_braking,
        settings.approxLevel
    )

    # conditions for diminishing section
    targetSpeedReached = drivingCourse[end][:v] <= 0.0
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]
    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R] #|| stateFlags[:tractionDeficit]
    #s_braking = brakingDistance(drivingCourse[end][:v], CS[:v_exit], train.a_braking, settings.approxLevel)
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] ||
                          stateFlags[:brakingStartReached]

    # use the conditions for the diminishing section
    if tractionDeficit && !targetSpeedReached && !endOfCSReached
        drivingMode = "diminishing"
        drivingCourse[end][:behavior] = drivingMode

        while tractionDeficit &&
                  !targetSpeedReached &&
                  !endOfCSReached &&
                  !brakingStartReached
            currentStepSize = settings.stepSize   # initialize the step size that can be reduced near intersections
            nextPoiPosition = getNextPoiPosition(
                CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPoiPosition

            for cycle in 1:(settings.approxLevel + 1)   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while tractionDeficit &&
                          !brakingStartReached &&
                          !pointOfInterestReached &&
                          !targetSpeedReached
                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(
                        drivingCourse[end][:F_T],
                        drivingCourse[end][:F_R],
                        train.m_train_full,
                        train.ξ_train
                    )

                    # create the next support point
                    push!(
                        drivingCourse,
                        moveAStep(
                            drivingCourse[end],
                            settings.stepVariable,
                            currentStepSize,
                            csId
                        )
                    )
                    drivingCourse[end][:behavior] = drivingMode

                    calculateForces!(
                        drivingCourse[end],
                        CSs,
                        csId,
                        drivingMode,
                        train,
                        settings.massModel
                    )

                    # conditions for the next while cycle
                    s_braking = brakingDistance(
                        drivingCourse[end][:v],
                        CS[:v_exit],
                        train.a_braking,
                        settings.approxLevel
                    )

                    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
                    pointOfInterestReached = drivingCourse[end][:s] >=
                                             nextPoiPosition
                    targetSpeedReached = drivingCourse[end][:v] <= 0.0
                    tractionDeficit = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
                end #while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel + 1
                    if drivingCourse[end][:v] < 0.0
                        #    if settings.stepVariable == :velocity
                        #        currentStepSize = drivingCourse[end-1][:v]
                        #    else
                        currentStepSize = settings.stepSize / 10.0^cycle
                        #    end
                    elseif drivingCourse[end][:F_T] > drivingCourse[end][:F_R]
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: F_T=",
                        drivingCourse[end][:F_T],
                        " > F_R=",
                        drivingCourse[end][:F_R]
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif s_braking > 0.0 &&
                           drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        "+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " > s_exit=", CS[:s_exit]
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " > nextPOI=",
                        nextPoiPosition
                        if settings.stepVariable == :distance
                            currentStepSize = nextPoiPosition -
                                              drivingCourse[end - 1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        "+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " == s_exit=", CS[:s_exit]
                        break

                    elseif drivingCourse[end][:s] == nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " == nextPOI=",
                        nextPoiPosition
                        break

                    elseif drivingCourse[end][:F_T] == drivingCourse[end][:F_R]
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: F_T=",
                        drivingCourse[end][:F_T],
                        " == F_R=",
                        drivingCourse[end][:F_R]
                        break

                    elseif drivingCourse[end][:v] == 0.0
                        @error "The train stops during diminishing run in CS",
                        csId,
                        " at position s=",
                        drivingCourse[end][:s],
                        " m because the maximum tractive effort is lower than the resistant forces.",
                        "Before the stop the last point has the values s=",
                        drivingCourse[end - 1][:s],
                        " m  v=",
                        drivingCourse[end - 1][:v],
                        " m/s  a=",
                        drivingCourse[end - 1][:a],
                        " m/s^2",
                        "F_T=",
                        drivingCourse[end - 1][:F_T],
                        " N  R_traction=",
                        drivingCourse[end - 1][:R_traction],
                        " N  R_wagons=",
                        drivingCourse[end - 1][:R_wagons],
                        " N  R_path=",
                        drivingCourse[end - 1][:R_path],
                        " N"

                    else
                        @error "ERROR during diminishing run: With the step variable ",
                        settings.stepVariable,
                        " the while loop will be left although s+s_braking<s_exit && v>0.0  in CS",
                        csId,
                        "  with s=",
                        drivingCourse[end][:s],
                        " m and v=",
                        drivingCourse[end][:v],
                        " m/s"
                    end
                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    brakingStartReached = false
                    pointOfInterestReached = false
                    targetSpeedReached = false
                    tractionDeficit = true

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " <= 0.0"
                        @error "ERROR: The train stops during diminishing run in CS",
                        csId,
                        " because the maximum tractive effort is lower than the resistant forces.",
                        "Before the stop the last point has the values s=",
                        drivingCourse[end - 1][:s],
                        " m  v=",
                        drivingCourse[end - 1][:v],
                        " m/s  a=",
                        drivingCourse[end - 1][:a],
                        " m/s^2",
                        "F_T=",
                        drivingCourse[end - 1][:F_T],
                        " N  R_traction=",
                        drivingCourse[end - 1][:R_traction],
                        " N  R_wagons=",
                        drivingCourse[end - 1][:R_wagons],
                        " N  R_path=",
                        drivingCourse[end - 1][:R_path],
                        " N."

                    elseif s_braking > 0.0 &&
                           drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        "+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " > s_exit=", CS[:s_exit]
                        pop!(drivingCourse)

                        pointOfInterestReached = false
                        targetSpeedReached = false
                        tractionDeficit = true

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " > nextPoiPosition=",
                        nextPoiPosition
                        drivingCourse[end][:s] = nextPoiPosition # round s down to nextPoiPosition

                    elseif drivingCourse[end][:F_T] >= drivingCourse[end][:F_R]
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: F_T=",
                        drivingCourse[end][:F_T],
                        " >= F_R=",
                        drivingCourse[end][:F_R]
                        break

                    else
                        @logmsg Trace "in CS",
                        csId,
                        " diminishing cycle",
                        cycle,
                        " case: else with v=",
                        drivingCourse[end][:v],
                        " > 0.0   and F_T=",
                        drivingCourse[end][:F_T],
                        " <= F_R=",
                        drivingCourse[end][:F_R]
                        @logmsg Trace (
                            "     and s +s_braking=",
                            drivingCourse[end][:s],
                            "+",
                            s_braking,
                            " = ",
                            drivingCourse[end][:s] + s_braking,
                            " <= s_exit=",
                            CS[:s_exit]
                        )
                        @logmsg Trace (
                            "     and s=",
                            drivingCourse[end][:s],
                            " <= nextPoiPosition=",
                            nextPoiPosition
                        )
                    end #if
                end #if
            end #for

            endOfCSReached = drivingCourse[end][:s] == CS[:s_exit]
        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = tractionDeficit
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_limit]
    stateFlags[:error] = !(endOfCSReached || brakingStartReached || !tractionDeficit)

    return (drivingCourse, stateFlags)
end #function addDiminishingSection!

"""
    addCoastingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the coasting section and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addClearingSection!`](@ref), [`addAcceleratingSection!`](@ref), [`addCruisingSection!`](@ref), [`addDiminishingSection!`](@ref), [`addBrakingSection!`](@ref), [`addHalt!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addCoastingSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    CS = CSs[csId]

    # conditions for coasting section
    lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
    previousSpeedLimitReached = lowestSpeedLimit[:v] != CS[:v_limit] &&
                                drivingCourse[end][:v] > lowestSpeedLimit[:v]
    speedLimitReached = drivingCourse[end][:v] > CS[:v_limit]
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit] ||
                         previousSpeedLimitReached ||
                         speedLimitReached
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]

    s_braking = brakingDistance(
        drivingCourse[end][:v],
        CS[:v_exit],
        train.a_braking,
        settings.approxLevel
    )
    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit] ||
                          stateFlags[:brakingStartReached]

    # use the conditions for the coasting section
    if !targetSpeedReached && !endOfCSReached
        drivingMode = "coasting"
        drivingCourse[end][:behavior] = drivingMode

        while !targetSpeedReached && !endOfCSReached && !brakingStartReached
            currentStepSize = settings.stepSize  # initialize the step size that can be reduced near intersections
            nextPoiPosition = getNextPoiPosition(
                CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPoiPosition

            for cycle in 1:(settings.approxLevel + 1)   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !targetSpeedReached && !brakingStartReached && !pointOfInterestReached
                    if drivingCourse[end][:s] >= lowestSpeedLimit[:s_end]
                        # could be asked after creating an support point. This way here prevents even a minimal exceedance of speed limit.
                        lowestSpeedLimit = getLowestSpeedLimit(
                            CSs,
                            csId,
                            drivingCourse[end][:s],
                            train.length
                        )
                    end

                    # traction effort and resisting forces (in N):
                    calculateForces!(
                        drivingCourse[end],
                        CSs,
                        csId,
                        drivingMode,
                        train,
                        settings.massModel
                    )

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = acceleration(
                        drivingCourse[end][:F_T],
                        drivingCourse[end][:F_R],
                        train.m_train_full,
                        train.ξ_train
                    )

                    # create the next support point
                    push!(
                        drivingCourse,
                        moveAStep(
                            drivingCourse[end],
                            settings.stepVariable,
                            currentStepSize,
                            csId
                        )
                    )
                    drivingCourse[end][:behavior] = drivingMode

                    # conditions for the next while cycle
                    s_braking = brakingDistance(
                        drivingCourse[end][:v],
                        CS[:v_exit],
                        train.a_braking,
                        settings.approxLevel
                    )
                    brakingStartReached = drivingCourse[end][:s] + s_braking >= CS[:s_exit]
                    pointOfInterestReached = drivingCourse[end][:s] >=
                                             nextPoiPosition
                    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit] ||
                                         drivingCourse[end][:v] > CS[:v_limit] ||
                                         lowestSpeedLimit[:v] < CS[:v_limit] && (
                                             drivingCourse[end][:v] >
                                             lowestSpeedLimit[:v] || (
                                             drivingCourse[end][:v] ==
                                             lowestSpeedLimit[:v] &&
                                             drivingCourse[end][:s] <
                                             lowestSpeedLimit[:s_end]
                                         )
                                         )
                end # while

                testFlag = false

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel + 1
                    if drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        "+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " > s_exit=", CS[:s_exit]
                        currentStepSize = settings.stepSize / 10.0^cycle

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: s=",
                        drivingCourse[end][:s],
                        " > nextPoiPosition=",
                        nextPoiPosition
                        if settings.stepVariable == :distance
                            currentStepSize = nextPoiPosition -
                                              drivingCourse[end - 1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] < CS[:v_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " < v_exit=",
                        CS[:v_exit]
                        if settings.stepVariable == :velocity
                            currentStepSize = drivingCourse[end - 1][:v] - CS[:v_exit]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] > lowestSpeedLimit[:v]
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " > v_lowestLimit=",
                        lowestSpeedLimit[:v]
                        if settings.stepVariable == :velocity
                            currentStepSize = lowestSpeedLimit[:v] -
                                              drivingCourse[end - 1][:v]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] + s_braking == CS[:s_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: s +s_braking=",
                        drivingCourse[end][:s],
                        "+",
                        s_braking,
                        " = ",
                        drivingCourse[end][:s] +s_braking, " == s_exit=", CS[:s_exit]
                        break

                    elseif drivingCourse[end][:v] == CS[:v_exit]
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: v=",
                        drivingCourse[end][:v],
                        " == v_exit=",
                        CS[:v_exit]
                        break

                    elseif drivingCourse[end][:s] == nextPoiPosition
                        @logmsg Trace "in CS",
                        csId,
                        " coasting cycle",
                        cycle,
                        " case: s =",
                        drivingCourse[end][:s],
                        " > nextPoiPosition=",
                        nextPoiPosition
                        break

                    else
                    end
                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    brakingStartReached = false
                    pointOfInterestReached = false
                    targetSpeedReached = false

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] <= 0.0
                        @info (
                            "The train stops during the coasting section in CS",
                            csId,
                            "                                                                ",
                            "       Before the stop the last point has the values s=",
                            drivingCourse[end - 1][:s],
                            " m  v=",
                            drivingCourse[end - 1][:v],
                            " m/s  a=",
                            drivingCourse[end - 1][:a],
                            " m/s^2",
                            "       F_T=",
                            drivingCourse[end - 1][:F_T],
                            " N  R_traction=",
                            drivingCourse[end - 1][:R_traction],
                            " N  R_wagons=",
                            drivingCourse[end - 1][:R_wagons],
                            " N  R_path=",
                            drivingCourse[end - 1][:R_path],
                            " N and s_braking=",
                            s_braking,
                            "m."
                        )

                    elseif drivingCourse[end][:s] + s_braking > CS[:s_exit]
                        # delete last support point because it went to far
                        pop!(drivingCourse)

                        # conditions for the next for cycle
                        #   brakingStartReached = true
                        pointOfInterestReached = false
                        targetSpeedReached = false

                    elseif drivingCourse[end][:v] > lowestSpeedLimit[:v] # if the train gets to fast it has to brake to hold the velocity limit
                        # delete last support point because it went to far
                        pop!(drivingCourse)

                        # conditions for the next for cycle
                        brakingStartReached = false
                        pointOfInterestReached = false
                        if lowestSpeedLimit[:v] != CS[:v_limit]
                            previousSpeedLimitReached = true
                        else
                            speedLimitReached = true
                        end

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        drivingCourse[end][:s] = nextPoiPosition # round s down to nextPoiPosition
                    else
                        # do nothing for example for drivingCourse[end][:s] + s_braking == CS[:s_exit]
                    end
                end
            end #for

            endOfCSReached = drivingCourse[end][:s] == CS[:s_exit]
        end #while
    end

    # set state flags
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:brakingStartReached] = brakingStartReached
    stateFlags[:tractionDeficit] = drivingCourse[end][:F_T] < drivingCourse[end][:F_R]
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0
    stateFlags[:previousSpeedLimitReached] = previousSpeedLimitReached
    stateFlags[:speedLimitReached] = speedLimitReached
    stateFlags[:error] = !(
        endOfCSReached ||
        brakingStartReached ||
        stateFlags[:tractionDeficit] ||
        previousSpeedLimitReached ||
        speedLimitReached
    )

    return (drivingCourse, stateFlags)
end #function addCoastingSection!

"""
    addBrakingSection!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the braking section and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addClearingSection!`](@ref), [`addAcceleratingSection!`](@ref), [`addCruisingSection!`](@ref), [`addDiminishingSection!`](@ref), [`addCoastingSection!`](@ref), [`addHalt!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addBrakingSection!(
        drivingCourse::Vector{Dict},
        stateFlags::Dict,
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    CS = CSs[csId]

    # conditions for braking section
    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit] || stateFlags[:endOfCSReached]

    # use the conditions for the braking section
    if !targetSpeedReached && !endOfCSReached
        drivingMode = "braking"
        drivingCourse[end][:behavior] = drivingMode

        while !targetSpeedReached && !endOfCSReached
            currentStepSize = settings.stepSize  # initialize the step size that can be reduced near intersections
            nextPoiPosition = getNextPoiPosition(
                CS[:pointsOfInterest], drivingCourse[end][:s])
            pointOfInterestReached = drivingCourse[end][:s] >= nextPoiPosition

            for cycle in 1:(settings.approxLevel + 1)   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while !targetSpeedReached && !endOfCSReached && !pointOfInterestReached
                    # traction effort and resisting forces (in N):
                    calculateForces!(
                        drivingCourse[end],
                        CSs,
                        csId,
                        drivingMode,
                        train,
                        settings.massModel
                    )

                    # acceleration (in m/s^2):
                    drivingCourse[end][:a] = train.a_braking

                    # Check if the train stops during this braking step. Therefore check if parts of square roots in moveAStep will be <0.0.
                    # If it ist the case calculate the exact braking acceleration for the fix end position and velocity.
                    if settings.stepVariable == :distance &&
                       (
                        (drivingCourse[end][:v] / drivingCourse[end][:a])^2 +
                        2 * currentStepSize / drivingCourse[end][:a]
                    ) < 0.0 ||
                       (
                        drivingCourse[end][:v]^2 +
                        2 * currentStepSize * drivingCourse[end][:a]
                    ) < 0.0
                        # create empty support point and set it for the values of s_exit and v_exit
                        push!(drivingCourse, SupportPoint())
                        drivingCourse[end][:behavior] = drivingMode
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse,
                            CS[:s_exit],
                            CS[:v_exit]
                        )
                    else
                        # create the next support point
                        push!(
                            drivingCourse,
                            moveAStep(
                                drivingCourse[end],
                                settings.stepVariable,
                                currentStepSize,
                                csId
                            )
                        )
                        drivingCourse[end][:behavior] = drivingMode
                    end

                    # conditions for the next while cycle
                    pointOfInterestReached = drivingCourse[end][:s] >=
                                             nextPoiPosition
                    endOfCSReached = drivingCourse[end][:s] >= CS[:s_exit]
                    targetSpeedReached = drivingCourse[end][:v] <= CS[:v_exit]
                end # while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < settings.approxLevel + 1
                    if drivingCourse[end][:v] < CS[:v_exit]
                        if settings.stepVariable == :velocity
                            currentStepSize = drivingCourse[end - 1][:v] - CS[:v_exit]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:s] > nextPoiPosition
                        if settings.stepVariable == :distance
                            currentStepSize = nextPoiPosition -
                                              drivingCourse[end - 1][:s]
                        else
                            currentStepSize = settings.stepSize / 10.0^cycle
                        end

                    elseif drivingCourse[end][:v] == CS[:v_exit] &&
                           drivingCourse[end][:s] == CS[:s_exit]
                        break

                    elseif drivingCourse[end][:v] == CS[:v_exit]
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse,
                            CS[:s_exit],
                            CS[:v_exit]
                        )
                        endOfCSReached = true
                        break

                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse,
                            CS[:s_exit],
                            CS[:v_exit]
                        )
                        targetSpeedReached = true
                        break

                    elseif drivingCourse[end][:s] == nextPoiPosition
                        break
                    end

                    # delete last support point for recalculating the last step with reduced step size
                    pop!(drivingCourse)

                    # conditions for the next for cycle
                    pointOfInterestReached = false
                    endOfCSReached = false
                    targetSpeedReached = false

                else # if the level of approximation is reached
                    if drivingCourse[end][:v] < 0.0
                        @logmsg Trace (
                            "during braking section in CS",
                            csId,
                            ": rounding v up from ",
                            drivingCourse[end][:v],
                            " to ",
                            CS[:v_exit]
                        ) # for testing
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse, CS[:s_exit], 0.0)
                        endOfCSReached = true
                        break
                    elseif drivingCourse[end][:s] > CS[:s_exit]
                        @logmsg Trace (
                            "during braking section in CS",
                            csId,
                            ": rounding s down from ",
                            drivingCourse[end][:s],
                            " to ",
                            CS[:s_exit]
                        ) # for testing
                        drivingCourse[end][:s] = CS[:s_exit]
                        break
                    elseif drivingCourse[end][:s] > nextPoiPosition
                        drivingCourse[end][:s] = nextPoiPosition # round s down to nextPoiPosition
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit] &&
                           drivingCourse[end][:s] == CS[:s_exit]
                        break
                    elseif drivingCourse[end][:v] < CS[:v_exit]
                        # reset last point with setting v=v_exit
                        @logmsg Trace (
                            "during braking section in CS",
                            csId,
                            ": rounding s up from ",
                            drivingCourse[end][:s],
                            " to ",
                            CS[:s_exit]
                        ) # for testing
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse,
                            CS[:s_exit],
                            CS[:v_exit]
                        )
                        endOfCSReached = true
                        break
                    elseif drivingCourse[end][:v] == CS[:v_exit]
                        @logmsg Trace (
                            "during braking section in CS",
                            csId,
                            ": rounding s up from ",
                            drivingCourse[end][:s],
                            " to ",
                            CS[:s_exit]
                        ) # for testing
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse,
                            CS[:s_exit],
                            CS[:v_exit]
                        )
                        endOfCSReached = true
                        break
                    elseif drivingCourse[end][:s] == CS[:s_exit]
                        @logmsg Trace (
                            "during braking section in CS",
                            csId,
                            ": rounding v down from ",
                            drivingCourse[end][:v],
                            " to ",
                            CS[:v_exit]
                        ) # for testing
                        drivingCourse = recalculateLastBrakingPoint!(
                            drivingCourse,
                            CS[:s_exit],
                            CS[:v_exit]
                        )
                        targetSpeedReached = true
                        break
                    else
                        # do nothing for example for drivingCourse[end][:s]==nextPoiPosition
                    end
                end
            end #for
        end #while
    end  # else: return the characteristic section without a braking section

    # set state flags
    lowestSpeedLimit = getLowestSpeedLimit(CSs, csId, drivingCourse[end][:s], train.length)
    stateFlags[:previousSpeedLimitReached] = lowestSpeedLimit[:v] != CS[:v_limit] &&
                                             drivingCourse[end][:v] >= lowestSpeedLimit[:v]
    stateFlags[:speedLimitReached] = drivingCourse[end][:v] >= CS[:v_exit]
    stateFlags[:endOfCSReached] = endOfCSReached
    stateFlags[:error] = !(endOfCSReached)
    calculateForces!(drivingCourse[end], CSs, csId, "default", train, settings.massModel)
    stateFlags[:resistingForceNegative] = drivingCourse[end][:F_R] < 0

    return (drivingCourse, stateFlags)
end #function addBrakingSection!

"""
    addHalt!(drivingCourse, stateFlags, CSs, csId, settings, train)

Calculate the support points of the halt section and add them to `drivingCourse`.

See also [`addBreakFreeSection!`](@ref), [`addClearingSection!`](@ref), [`addAcceleratingSection!`](@ref), [`addCruisingSection!`](@ref), [`addDiminishingSection!`](@ref), [`addCoastingSection!`](@ref), [`addBrakingSection!`](@ref).

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `stateFlags::Dict`: the Dictionary containing different states for `drivingCourse`'s values.
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `csId::Integer`: the number of characteristic section in the Vector `CSs`.
- `settings::Settings`: the Settings object containing settings used for calculation.
- `train::Train`: the Train object containing attributes with technical train data.

# Examples
See [`addBreakFreeSection!`](@ref).
"""
function addHalt!(
        drivingCourse::Vector{Dict},
        CSs::Vector{Dict},
        csId::Integer,
        settings::Settings,
        train::Train
)
    if drivingCourse[end][:v] == 0.0
        drivingMode = "halt"
        drivingCourse[end][:behavior] = drivingMode

        # traction effort and resisting forces (in N)
        calculateForces!(
            drivingCourse[end],
            CSs,
            csId,
            drivingMode,
            train,
            settings.massModel
        )
    end  # else: return the characteristic section without a halt section section
    return drivingCourse
end #function addHalt!

"""
    recalculateLastBrakingPoint!(drivingCourse, s_target, v_target)

Realculate the last step of the `drivingCourse` depending on new position and velocity.

# Arguments
- `drivingCourse::Vector{Dict}`: the Vector containing dictionaries for all support points.
- `s_target::Real`: the new positon for the last support point in m.
- `v_target::Real`: the new velocity for the last support point in m/s.

# Examples
```julia-repl
julia> recalculateLastBrakingPoint(drivingCourse_with_two_points_near_5000m, 5000.0, 23.333333333333332)
Dict{Symbol, Any}[Dict(:behavior => "braking", :F_T => 0.0, :R_wagons => 21655.0, :R_path => 0.0, :s => 4999.999980000006, :v => 23.333333441372545, :R_train => 28640.8, :R_traction => 6985.8, :a => -0.126, :t => 235.16501930395856, :F_R => 28640.8), Dict(:behavior => "braking", :F_T => 0.0, :R_wagons => 0.0, :R_path => 0.0, :s => 5000.0, :v => 23.333333333333332, :R_train => 0.0, :R_traction => 0.0, :a => 0.0, :t => 235.16502016110115, :F_R => 0.0)]
```
"""
function recalculateLastBrakingPoint!(
        drivingCourse::Vector{Dict},
        s_target::Real,
        v_target::Real
)
    currentPoint = drivingCourse[end]
    previousPoint = drivingCourse[end - 1]

    # set s and v
    currentPoint[:s] = s_target          # position (in m)
    currentPoint[:v] = v_target          # velocity (in m/s)

    # calculate other values
    previousPoint[:a] = brakingAcceleration(
        previousPoint[:v],
        currentPoint[:v],
        currentPoint[:s] - previousPoint[:s]
    )
    currentPoint[:t] = previousPoint[:t] +
                       Δt_with_Δv(currentPoint[:v] - previousPoint[:v], previousPoint[:a])             # point in time (in s)

    return drivingCourse
end #function recalculateLastBrakingPoint

"""
    secureBrakingBehavior!(CSs, a_braking, approxLevel)

Calculate the velocities between the characterisitc sections to secure braking behavior.

# Arguments
- `CSs::Vector{Dict}`: the Vector containing dictionaries for all characteristic sections.
- `a_braking::Real`: the constant braking acceleration in m/s^2.
- `approxLevel::Integer`: the last position behind the decimal point that is not rounded.

# Examples
```julia-repl
julia> secureBrakingBehavior!(CSs, -0.375, 3)
Dict[Dict{Symbol, Any}(:s_entry => 0.0, :v_exit => 19.3649, :v_limit => 25.0, :r_path => 0.0, :pointsOfInterest => NamedTuple[(s = 9500.0, label = "")], :s_exit => 9500.0), Dict{Symbol, Any}(:s_entry => 0.0, :v_exit => 0.0, :v_limit => 25.0, :r_path => 1.0, :pointsOfInterest => NamedTuple[(s = 10000.0, label = "")], :s_exit => 10000.0)]
```
"""
function secureBrakingBehavior!(CSs::Vector{Dict}, a_braking::Real, approxLevel::Integer)
    # limit the entry and exit velocities of the characteristic sections to secure that the train stops at the moving sections end

    csId = length(CSs)
    v_entryFollowing = 0.0     # the exit velocity of the last characteristic section is 0.0 m/s
    while csId >= 1
        # calculate the maximum possible entry velocity to define the previous section's maximum allowed exit velocity
        CS = CSs[csId]

        CS[:v_exit] = min(CS[:v_limit], v_entryFollowing)

        v_entry = brakingStartVelocity(
            CS[:v_exit],
            a_braking,
            CS[:s_exit] - CS[:s_entry],
            approxLevel
        )

        v_entryFollowing = min(CS[:v_limit], v_entry)

        csId = csId - 1
    end #while
    return CSs
end #function secureBrakingBehavior!
