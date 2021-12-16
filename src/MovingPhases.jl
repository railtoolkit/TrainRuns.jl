module MovingPhases

using ..types
export addAccelerationPhase!, addAccelerationPhaseUntilBraking!, addCruisingPhase!, addCoastingPhaseUntilBraking!, addBrakingPhase!, addBrakingPhaseStepwise!, calculateForces!
# addBrakingPhaseStepwise! is not used in the current version of the tool

v00=100/3.6     # velocity constant (in m/s)
g=9.81          # acceleration due to gravity (in m/s^2)            # TODO: should more digits of g be used?  g=9,80665 m/s^2

approximationLevel = 6  # value for approximation to intersections TODO further explanation (e.g. approximationLevel = 3 -> with stepSize 10 m the approximation will be calculated accurate on 10 mm ; 1s -> 1 ms; 1 km/h -> 3.6 mm/s)
    # TODO: define it in TrainRun and give it to each function?

## functions for calculating tractive effort and resisting forces
"""
    calculateTractiveEffort(v, tractiveEffortVelocityPairs)

Calculate the trains tractive effort with the `tractiveEffortVelocityPairs` dependend on the velocity `v`.

...
# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `tractiveEffortVelocityPairs::Array{Array{AbstractFloat,1},1}`: the trains pairs for velocity in m/s and tractive effort in N as one array containing an array for each pair.
...

# Examples
```julia-repl
julia> calculateTractiveEffort(20.0, [[0.0, 180000], [20.0, 100000], [40.0, 60000], [60.0, 40000], [80.0, 30000]])
100000

julia> calculateTractiveEffort(30.0, [[0.0, 180000], [20.0, 100000], [40.0, 60000], [60.0, 40000], [80.0, 30000]])
80000
```
"""
function calculateTractiveEffort(v::AbstractFloat, tractiveEffortVelocityPairs)
    for row in 1:length(tractiveEffortVelocityPairs)
        if  tractiveEffortVelocityPairs[row][1]==v
            return tractiveEffortVelocityPairs[row][2]
        elseif tractiveEffortVelocityPairs[row][1]>v
            # interpolate for a straight line between the two surrounding points with the formula: F=(v-v_(row-1))*(F_row-F_(row-1))/(v_row-v_(row-1))+F_(row-1)
            F_T_interpolation=(v-tractiveEffortVelocityPairs[row-1][1])*(tractiveEffortVelocityPairs[row][2]-tractiveEffortVelocityPairs[row-1][2])/(tractiveEffortVelocityPairs[row][1]-tractiveEffortVelocityPairs[row-1][1])+tractiveEffortVelocityPairs[row-1][2]
            return F_T_interpolation
        end #if
    end #for
    # if v gets higher than the velocities in tractiveEffortVelocityPairs the last tractive effort will be used
        # TODO: also an extrapolation could be used
    return tractiveEffortVelocityPairs[end][2]
end #function calculateTractiveEffort


#TODO: choose an explanation and replace the ? ? ?
"""
    calculateTractionUnitResistance(v, train)

Calculate the traction units vehicle resistance dependend on the velocity `v`.
Calculate the vehicle resistance for the traction unit of the `train` dependend on the velocity `v`.

...
# Arguments
- `v::AbstractFloat`: the current velocity in m/s.
- `train::Dict`: ? ? ?
...

# Examples
```julia-repl
julia> calculateTractionUnitResistance(30.0, ? ? ?)
? ? ?
```
"""
function calculateTractionUnitResistance(v::AbstractFloat, train::Dict)
    return train[:f_Rtd0]/1000*train[:m_td]*g+train[:f_Rtc0]/1000*train[:m_tc]*g+train[:F_Rt2]*((v+train[:Δv_t])/v00)^2    # /1000 because of the unit ‰
end #function calculateTractionUnitResistance

"""
calculate and return the wagons vehicle resistance dependend on the velocity
"""
function calculateWagonsResistance(v::AbstractFloat, train::Dict)
    return train[:m_w]*g*(train[:f_Rw0]/1000+train[:f_Rw1]/1000*v/v00+train[:f_Rw2]/1000*((v+train[:Δv_w])/v00)^2)          # /1000 because of the unit ‰
end #function calculateWagonsResistance

"""
calculate and return the path resistance dependend on the trains position and mass model
"""
function calculatePathResistance(s::AbstractFloat, massModel::String, train::Dict, CSs::Vector{CharacteristicSection})
    # looking for the characteristic section with the trains head position
    id=length(CSs)
    while s<CSs[id].s_entry
        id=id-1
        if id==0
            println("s=",s,"   MS.s_entry=",CSs[1].s_entry)
        end
    end #while

    if massModel=="mass point"
        pathResistance=CSs[id].f_Rp/1000*train[:m_train]*g        # /1000 because of the unit ‰
    elseif massModel=="homogeneous strip"
        pathResistance=0.0
        while id>0 && s-train[:l_train]<CSs[id].s_exit
            pathResistance=pathResistance+(min(s, CSs[id].s_exit)-max(s-train[:l_train], CSs[id].s_entry))/train[:l_train]*(CSs[id].f_Rp/1000*train[:m_train]*g)      # /1000 because of the unit ‰
            id=id-1
            if id==0
                # TODO: currently for values  < movingSection[:s_entry] the values of movingSection[:s_entry]  will be used
                return pathResistance+(CSs[1].s_entry-(s-train[:l_train]))/train[:l_train]*(CSs[1].f_Rp/1000*train[:m_train]*g)        # /1000 because of the unit ‰
            end #if
        end #while
    else
        error("ERROR at the calculation of path resistance: settings[:massModel]!=mass point && settings[:massModel]!=homogeneous strip")
    end #if

    return pathResistance
end #function pathResistance

"""
calculate and return tractive and resisting forces for a data point
"""
function calculateForces!(dataPoint::DataPoint, train::Dict, massModel::String,  CSs::Vector{CharacteristicSection}, bsType::String)
    # calculate resisting forces
    dataPoint.R_traction=calculateTractionUnitResistance(dataPoint.v, train)
    dataPoint.R_consist=calculateWagonsResistance(dataPoint.v, train)
    dataPoint.R_train=dataPoint.R_traction+dataPoint.R_consist
    dataPoint.R_path=calculatePathResistance(dataPoint.s, massModel, train, CSs)
    dataPoint.F_R=dataPoint.R_train+dataPoint.R_path

    # calculate tractive effort
    if bsType == "acceleration" || bsType == "diminishing"
        dataPoint.F_T = calculateTractiveEffort(dataPoint.v, train[:tractiveEffortVelocityPairs])
    elseif bsType == "cruising"
        dataPoint.F_T = min(max(0.0, dataPoint.F_R), calculateTractiveEffort(dataPoint.v, train[:tractiveEffortVelocityPairs]))
    else
        dataPoint.F_T = 0.0
    end
    return dataPoint
end #function calculateForces


"""
TODO
"""
function moveAStep(previousPoint::DataPoint, stepVariable::String, stepSize::Real, csId::Integer)
 # 12/15 old with float stepsize function moveAStep(previousPoint::DataPoint, stepVariable::String, stepSize::AbstractFloat, csId::Integer)
    # stepSize is the currentStepSize depending on the accessing function
    # TODO: csId is only for error messages. Should it be removed?
    #= 08/31 TODO: How to check if the train stopps during this step? I should throw an error myself that I catch in higher hierarchies.    =#

    # creating the next data point
    newPoint=DataPoint()
    newPoint.i=previousPoint.i+1         # identifier

    # calculate s, t, v, E
    if previousPoint.a==0.0 # TODO: or better stepVariable=="s_cruising in m" ?
       newPoint.Δs=stepSize          # step size (in m)
       newPoint.Δt=newPoint.Δs/previousPoint.v      # step size (in s)
       newPoint.Δv=0.0                                               # step size (in m/s)
    elseif stepVariable=="s in m"                                                           # distance step method
        newPoint.Δs=stepSize                                                                # step size (in m)
        # 11/21 |->
        if previousPoint.a<0.0
            if ((previousPoint.v/previousPoint.a)^2+2*newPoint.Δs/previousPoint.a)<0.0 || (previousPoint.v^2+2*newPoint.Δs*previousPoint.a)<0.0  # checking if the parts of the following square roots will be <0.0
                error("ERROR: The train stops during the acceleration phase in CS",csId," because the tractive effort is lower than the resistant forces.",
                "       Before the stop the last point has the values s=",previousPoint.s," m,  v=",previousPoint.v," m/s,  a=",previousPoint.a," m/s^2,",
                "       F_T=",previousPoint.F_T," N,  R_traction=",previousPoint.R_traction," N,  R_consist=",previousPoint.R_consist," N,  R_path=",previousPoint.R_path," N.")
            end
        end
        # 11/21 ->|
        newPoint.Δt=sign(previousPoint.a)*sqrt((previousPoint.v/previousPoint.a)^2+2*newPoint.Δs/previousPoint.a)-previousPoint.v/previousPoint.a                          # step size (in s)
        newPoint.Δv=sqrt(previousPoint.v^2+2*newPoint.Δs*previousPoint.a)-previousPoint.v   # step size (in m/s)
    elseif stepVariable=="t in s"                                                       # time step method
        newPoint.Δt=stepSize                                                                # step size (in s)
        newPoint.Δs=newPoint.Δt*(2*previousPoint.v+newPoint.Δt*previousPoint.a)/2           # step size (in m)
        newPoint.Δv=newPoint.Δt*previousPoint.a                                             # step size (in m/s)
    elseif stepVariable=="v in m/s"                                                     # velocity step method
        newPoint.Δv=stepSize*sign(previousPoint.a)                                          # step size (in m/s)
        newPoint.Δs=((previousPoint.v+newPoint.Δv)^2-previousPoint.v^2)/2/previousPoint.a   # step size (in m)
        newPoint.Δt=newPoint.Δv/previousPoint.a                                             # step size (in s)
    end #if

    newPoint.s=previousPoint.s+newPoint.Δs                            # position (in m)
    newPoint.t=previousPoint.t+newPoint.Δt                            # point in time (in s)
    newPoint.v=previousPoint.v+newPoint.Δv                            # velocity (in m/s)
    newPoint.ΔW=previousPoint.F_T*newPoint.Δs                       # mechanical work in this step (in Ws)
    newPoint.W=previousPoint.W+newPoint.ΔW                      # mechanical work (in Ws)
    newPoint.ΔE=newPoint.ΔW                                         # energy consumption in this step (in Ws)
    newPoint.E=previousPoint.E+newPoint.ΔE                            # energy consumption (in Ws)


    return newPoint
end #function moveAStep


"""
# if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
"""
function detectFormerSpeedLimits(CSs::Vector{CharacteristicSection}, csWithTrainHeadId::Integer, currentPoint::DataPoint, l_train::Real)
    formerSpeedLimits=[]
    if csWithTrainHeadId > 1 && currentPoint.s - l_train < CSs[csWithTrainHeadId].s_entry
        formerCsId=csWithTrainHeadId-1
        while formerCsId > 0 && currentPoint.s - l_train < CSs[formerCsId].s_exit
            if CSs[formerCsId].v_limit < CSs[csWithTrainHeadId].v_limit    # TODO: is the position of trains tail < movingSection[:s_entry], v_limit of the first CS is used
                push!(formerSpeedLimits, [CSs[formerCsId].s_exit, CSs[formerCsId].v_limit])
                for i in 1:length(formerSpeedLimits)-1
                    if formerSpeedLimits[i][2]<=formerSpeedLimits[end][2]
                        pop!(formerSpeedLimits)
                        break
                    end
                end
            end
            formerCsId=formerCsId-1
        end
    end
    return formerSpeedLimits
end # function detectFormerSpeedLimits

function considerFormerSpeedLimits!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection}, formerSpeedLimits, accelerationSection::BehaviorSection)
 #  TODO: What is the type of formerSpeedLimits?   function considerFormerSpeedLimits!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection}, formerSpeedLimits::Array{Array{AbstractFloat,1},1}, accelerationSection::BehaviorSection)
 #        would work: function considerFormerSpeedLimits!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection}, formerSpeedLimits::Array{Any,1}, accelerationSection::BehaviorSection)
    if length(formerSpeedLimits) > 0
        # if a former speed limit has been exceeded the acceleration steps of this CS will be removed and a clearing phase will be inserted before acceleration
        if drivingCourse[end].v > formerSpeedLimits[end][2]
            while drivingCourse[end].s > get(CS.behaviorSections, :clearing, accelerationSection).s_entry
                pop!(drivingCourse)
            end

            if haskey(CS.behaviorSections, :clearing)
                CS.t = CS.t-CS.behaviorSections[:clearing].t              # reducing the total running time (in s)
                CS.E = CS.E-CS.behaviorSections[:clearing].E              # reducing the total energy consumption (in Ws)
                delete!(CS.behaviorSections, :clearing)
            end

            # create a (new and longer) clearing section
            s_braking=max(0.0, ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))
            s_clearing=min(CS.s_exit-drivingCourse[end].s-s_braking, formerSpeedLimits[end][1]-(drivingCourse[end].s-train[:l_train]))

            if s_clearing>0.0
                (CS, drivingCourse)=addCruisingPhase!(CS, drivingCourse, s_clearing, settings, train, CSs, "clearing")
            else
                error("ERROR: clearing <=0.0 although it has to be >0.0 in CS ",CS.id)
            end

            # 09/22: if  drivingCourse[end].s < CS.s_exit
            if  drivingCourse[end].s < CS.s_exit-s_braking
                # reset the accelerationSection
                accelerationSection=BehaviorSection()
                accelerationSection.type="acceleration"             # type of behavior section
                accelerationSection.s_entry=drivingCourse[end].s      # first position (in m)
                accelerationSection.v_entry=drivingCourse[end].v      # entry speed (in m/s)

                #currentStepSize=settings[:stepSize]  # initializing the step size that can be reduced near intersections
            else
                return (CS, drivingCourse, formerSpeedLimits, accelerationSection, true)
            end
        end

        # remove former speed limits of characteristic sections the train has left during the last step from the list
        while length(formerSpeedLimits) > 0 && drivingCourse[end].s - train[:l_train] >= formerSpeedLimits[end][1]
            pop!(formerSpeedLimits)
        end
    end
    return (CS, drivingCourse, formerSpeedLimits, accelerationSection, false)
end # function considerFormerSpeedLimits!

## This function calculates the data points of the breakFree phase.
# Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for breakFree if needed.
# Info: currently the values of the breakFree phase will be calculated like in the acceleration phase
function addStartingPhase!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection})
    if drivingCourse[end].v==0.0 && drivingCourse[end].s<CS.s_exit
        breakFreeSection=BehaviorSection()
        breakFreeSection.type="breakFree"                        # type of behavior section
        breakFreeSection.s_entry=drivingCourse[end].s            # first position (in m)
        breakFreeSection.v_entry=drivingCourse[end].v            # entry speed (in m/s)
        push!(breakFreeSection.dataPoints, drivingCourse[end].i) # list of containing data points

        # traction effort and resisting forces (in N):
        drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "acceleration"))    # currently the tractive effort is calculated like in the acceleration phase

        # acceleration (in m/s^2):
        drivingCourse[end].a=(drivingCourse[end].F_T-drivingCourse[end].F_R)/train[:m_train]/train[:ξ_train]
        if drivingCourse[end].a<0.0
            error("ERROR: a<0 m/s^2 in the breakFree phase !")
        elseif drivingCourse[end].a==0.0
            error("ERROR: a=0 m/s^2 in the breakFree phase !")
        end

        # creating the next data point
        push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], settings[:stepSize], CS.id))
           #= 07/30 TODO: the calculation is easier with these lines because the values that are 0 in the first step are not used in calculation but all in all the code gets easier without these lines:
            push!(drivingCourse, DataPoint())
            drivingCourse[end].i=drivingCourse[end-1].i+1     # incrementing the number of the data point

            # calculate s, t, v, E
            if settings[:stepVariable]=="s in m"                                                           # distance step method
                drivingCourse[end].Δs=settings[:stepSize]                                                     # step size (in m)
                drivingCourse[end].Δt=sqrt(2*drivingCourse[end].Δs/drivingCourse[end-1].a)                  # step size (in s)    (in this formula drivingCourse[end-1].v is missing because it is 0.0)
                drivingCourse[end].Δv=sqrt(2*drivingCourse[end].Δs*drivingCourse[end-1].a)                  # step size (in m/s)  (in this formula drivingCourse[end-1].v is missing because it is 0.0)
            elseif settings[:stepVariable]=="t in s"                                                       # time step method
                drivingCourse[end].Δt=settings[:stepSize]                                                     # step size (in s)
                drivingCourse[end].Δs=drivingCourse[end].Δt*(drivingCourse[end].Δt*drivingCourse[end-1].a)/2  # step size (in m)
                drivingCourse[end].Δv=drivingCourse[end].Δt*drivingCourse[end-1].a                          # step size (in m/s)
            elseif settings[:stepVariable]=="v in m/s"                                                     # velocity step method
                drivingCourse[end].Δv=settings[:stepSize]*sign(drivingCourse[end-1].a)                        # step size (in m/s)
                drivingCourse[end].Δs=drivingCourse[end].Δv^2/2/drivingCourse[end-1].a                      # step size (in m)
                drivingCourse[end].Δt=drivingCourse[end].Δv/drivingCourse[end-1].a                          # step size (in s)
            end #if

            #drivingCourse[end].s=ceil(drivingCourse[end-1].s+drivingCourse[end].Δs, digits=approximationLevel)          # position (in m)  # rounded -> exact to 1 nm
            drivingCourse[end].s=drivingCourse[end-1].s+drivingCourse[end].Δs                            # position (in m)
            drivingCourse[end].t=drivingCourse[end-1].t+drivingCourse[end].Δt                            # point in time (in s)
            drivingCourse[end].v=drivingCourse[end-1].v+drivingCourse[end].Δv                            # velocity (in m/s)
            drivingCourse[end].ΔW=drivingCourse[end-1].F_T*drivingCourse[end].Δs                       # mechanical work in this step (in Ws)
            drivingCourse[end].W=drivingCourse[end-1].W+drivingCourse[end].ΔW                      # mechanical work (in Ws)
            drivingCourse[end].ΔE=drivingCourse[end].ΔW                                                # energy consumption in this step (in Ws)
            drivingCourse[end].E=drivingCourse[end-1].E+drivingCourse[end].ΔE                            # energy consumption (in Ws)
            =#
        push!(breakFreeSection.dataPoints, drivingCourse[end].i)

        # calculate the accumulated breakFree section information
        breakFreeSection.s_exit=drivingCourse[end].s                                                  # last position (in m)
        breakFreeSection.length=breakFreeSection.s_exit-breakFreeSection.s_entry                       # total length  (in m)
        breakFreeSection.v_exit=drivingCourse[end].v                                                 # exit speed (in m/s)
        breakFreeSection.t=drivingCourse[end].t-drivingCourse[breakFreeSection.dataPoints[1]].t  # total running time (in s)
        breakFreeSection.E=drivingCourse[end].E-drivingCourse[breakFreeSection.dataPoints[1]].E  # total energy consumption (in Ws)

        CS.t=CS.t+breakFreeSection.t         # total running time (in s)
        CS.E=CS.E+breakFreeSection.E         # total energy consumption (in Ws)

        merge!(CS.behaviorSections, Dict(:breakFree => breakFreeSection))
    end # else: return the characteristic section without a breakFree section
    return (CS, drivingCourse)
end #function addStartingPhase!


## This function calculates the data points of the acceleration phase.
 #  Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the acceleration section
function addAccelerationPhase!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection})
    if drivingCourse[end].v==0.0
        (CS, drivingCourse)=addStartingPhase!(CS, drivingCourse, settings, train, CSs)
    end #if

    calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "acceleration")
    if drivingCourse[end].F_T < drivingCourse[end].F_R
        (CS, drivingCourse)=addDiminishingPhase!(CS, drivingCourse, settings, train, CSs)
        drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "acceleration"))
    end

    # if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
    formerSpeedLimits = detectFormerSpeedLimits(CSs, CS.id, drivingCourse[end], train[:l_train])

    # conditions for acceleration phase
    targetSpeedReached = drivingCourse[end].v >= CS.v_target
    trainAtEnd = drivingCourse[end].s >= CS.s_exit
    tractionSurplus = drivingCourse[end].F_T > drivingCourse[end].F_R

    # use the conditions for the acceleration phase
    if !targetSpeedReached && !trainAtEnd && tractionSurplus
    #11/23 long version:    if drivingCourse[end].v < CS.v_target && drivingCourse[end].s <CS.s_exit && drivingCourse[end].F_T > drivingCourse[end].F_R
        accelerationSection=BehaviorSection()
        accelerationSection.type="acceleration"               # type of behavior section
        accelerationSection.s_entry=drivingCourse[end].s      # first position (in m)
        accelerationSection.v_entry=drivingCourse[end].v      # entry speed (in m/s)
        push!(accelerationSection.dataPoints, drivingCourse[end].i)

        currentStepSize=settings[:stepSize]  # initializing the step size that can be reduced near intersections
        for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
            while drivingCourse[end].v<CS.v_target && drivingCourse[end].s<CS.s_exit && drivingCourse[end].F_T > drivingCourse[end].F_R
                # traction effort and resisting forces (in N)
                # 11/22   drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))

                # acceleration (in m/s^2):
                drivingCourse[end].a=(drivingCourse[end].F_T-drivingCourse[end].F_R)/train[:m_train]/train[:ξ_train]

                # create the next data point
                push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS.id))
                push!(accelerationSection.dataPoints, drivingCourse[end].i)
            #    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))


                if length(formerSpeedLimits) > 0 # If the tail of the train is located in a former characteristic section with lower speed limit check if is is possible to accelerate as normal
                    (CS, drivingCourse, formerSpeedLimits, accelerationSection, endOfCsReached) = considerFormerSpeedLimits!(CS, drivingCourse, settings, train, CSs, formerSpeedLimits, accelerationSection)
                    if  endOfCsReached
                        return (CS, drivingCourse)
                    end #if
                    currentStepSize=settings[:stepSize]  # initializing the step size that can be reduced near intersections
                end #if
                drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))
            end #while

            # check which limit was reached and adjust the currentStepSize for the next cycle
            if cycle < approximationLevel+1
                if drivingCourse[end].v<=0.0
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                elseif drivingCourse[end].s>CS.s_exit
                    if settings[:stepVariable] == "s in m"
                        currentStepSize=CS.s_exit-drivingCourse[end-1].s
                    else
                        currentStepSize = settings[:stepSize] / 10.0^cycle
                    end

                elseif drivingCourse[end].v>CS.v_target
                    if settings[:stepVariable]=="v in m/s"
                        currentStepSize=CS.v_target-drivingCourse[end-1].v
                    else
                        currentStepSize = settings[:stepSize] / 10.0^cycle
                    end

                elseif drivingCourse[end].s==CS.s_exit
                    break

                elseif drivingCourse[end].v==CS.v_target
                    break
                elseif drivingCourse[end].F_T <= drivingCourse[end].F_R
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                else
                    error("ERROR at acceleration phase: With the step variable ", settings[:stepVariable]," the while loop will be left although v<v_target and s<s_exit in CS",CS.id,"  with s=" ,drivingCourse[end].s," m and v=",drivingCourse[end].v," m/s")
                end
                # delete last data point for recalculating the last step with reduced step size
                pop!(drivingCourse)
                pop!(accelerationSection.dataPoints)

            else # if the level of approximation is reached
                if drivingCourse[end].v<=0.0
                    # push!(accelerationSection.dataPoints, drivingCourse[end].i)
                    error("ERROR: The train stops during the acceleration phase in CS",CS.id," because the tractive effort is lower than the resistant forces.",
                    "       Before the stop the last point has the values s=",drivingCourse[end-1].s," m  v=",drivingCourse[end-1].v," m/s  a=",drivingCourse[end-1].a," m/s^2",
                    "       F_T=",drivingCourse[end-1].F_T," N  R_traction=",drivingCourse[end-1].R_traction," N  R_consist=",drivingCourse[end-1].R_consist," N  R_path=",drivingCourse[end-1].R_path," N.")

                elseif drivingCourse[end].v>CS.v_target
                    pop!(drivingCourse)
                    pop!(accelerationSection.dataPoints)
                elseif drivingCourse[end].s>CS.s_exit
                    drivingCourse[end].s=CS.s_exit # rounding s down to s_exit

                elseif drivingCourse[end].F_T <= drivingCourse[end].F_R
                    (CS, drivingCourse)=addDiminishingPhase!(CS, drivingCourse, settings, train, CSs)
                    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))

                else

                end
            end
        end #for

        if length(accelerationSection.dataPoints) > 1 # 11/21 new: it is possible that the acceleration starts at v_target, accelerates a step, is to high and drops the last point. then there is only one data point which is not a section.
            # calculate the accumulated acceleration section information
            accelerationSection.v_exit=drivingCourse[end].v                                                     # exit speed (in m/s)
            accelerationSection.s_exit=drivingCourse[end].s                                                      # last position (in m)
            accelerationSection.length=accelerationSection.s_exit-accelerationSection.s_entry                   # total length  (in m)
            accelerationSection.t=drivingCourse[end].t-drivingCourse[accelerationSection.dataPoints[1]].t  # total running time (in s)
            accelerationSection.E=drivingCourse[end].E-drivingCourse[accelerationSection.dataPoints[1]].E  # total energy consumption (in Ws)
            CS.t=CS.t+accelerationSection.t             # total running time (in s)
            CS.E=CS.E+accelerationSection.E             # total energy consumption (in Ws)

            # TODO: this warning schould not be needed. just for testing
            if CS.v_target < drivingCourse[end].v
                println("WARNING, v is getting to high at the end of the acceleration phase.   v=",drivingCourse[end].v ,"  > v_target=",CS.v_target)
            end

            merge!(CS.behaviorSections, Dict(:acceleration=>accelerationSection))
        end
    end # else: just return the given data point number without changes due to the acceleration phase

    return (CS, drivingCourse)
end #function addAccelerationPhase!


## This function calculates the data points of the acceleration phase.
function addAccelerationPhaseUntilBraking!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection})
    if drivingCourse[end].v==0.0
        (CS, drivingCourse)=addStartingPhase!(CS, drivingCourse, settings, train, CSs)
    end #if

    calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "acceleration")
    if drivingCourse[end].F_T < drivingCourse[end].F_R
        (CS, drivingCourse)=addDiminishingPhase!(CS, drivingCourse, settings, train, CSs)
        drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "acceleration"))
    end

    # if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
    formerSpeedLimits = detectFormerSpeedLimits(CSs, CS.id, drivingCourse[end], train[:l_train])


    # 11/23 old without F_T>F_R: if drivingCourse[end].v < CS.v_target && drivingCourse[end].s<CS.s_exit
    if drivingCourse[end].v < CS.v_target && drivingCourse[end].s<CS.s_exit && drivingCourse[end].F_T > drivingCourse[end].F_R
        accelerationSection=BehaviorSection()
        accelerationSection.type="acceleration"                 # type of behavior section
        accelerationSection.s_entry=drivingCourse[end].s        # first position (in m)
        accelerationSection.v_entry=drivingCourse[end].v        # entry speed (in m/s)
        push!(accelerationSection.dataPoints, drivingCourse[end].i)

        currentStepSize=settings[:stepSize]   # initializing the step size that can be reduced near intersections
        for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
            s_braking=max(0.0, ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))
            while drivingCourse[end].v < CS.v_target && drivingCourse[end].s+s_braking<CS.s_exit && drivingCourse[end].F_T > drivingCourse[end].F_R      # as long as s_i + s_braking < s_CSend
            # 12/03 old with v>0    while drivingCourse[end].v < CS.v_target && drivingCourse[end].s+s_braking<CS.s_exit && drivingCourse[end].v>0.0 && drivingCourse[end].F_T > drivingCourse[end].F_R      # as long as s_i + s_braking < s_CSend

            #11/22    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))

                # acceleration (in m/s^2):
                drivingCourse[end].a=(drivingCourse[end].F_T-drivingCourse[end].F_R)/train[:m_train]/train[:ξ_train]
            #    if drivingCourse[end].a==0.0
            #        error("ERROR: a=0 m/s^2 in the acceleration phase !   with  F_T=",drivingCourse[end].F_T,"  R_traction=",drivingCourse[end].R_traction,"  R_consist=",drivingCourse[end].R_consist,"  R_path=",drivingCourse[end].R_path)
            #    end

                # create the next data point
                push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS.id))
                push!(accelerationSection.dataPoints, drivingCourse[end].i)
            # 12/03: was moved behind considerFormerSpeedLimits:    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))

                if length(formerSpeedLimits) > 0 # If the tail of the train is located in a former characteristic section with lower speed limit check if is is possible to accelerate as normal
                    (CS, drivingCourse, formerSpeedLimits, accelerationSection, endOfCsReached) = considerFormerSpeedLimits!(CS, drivingCourse, settings, train, CSs, formerSpeedLimits, accelerationSection)
                    if  endOfCsReached
                        return (CS, drivingCourse)
                    end
                end
                drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))
                s_braking=max(0.0, ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))
            end #while

            # check which limit was reached and adjust the currentStepSize for the next cycle
            if cycle < approximationLevel+1
                if drivingCourse[end].v<=0.0
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                elseif drivingCourse[end].s +s_braking > CS.s_exit
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                elseif drivingCourse[end].v>CS.v_target
                    if settings[:stepVariable]=="v in m/s"
                        currentStepSize= CS.v_target-drivingCourse[end-1].v
                    else
                        currentStepSize = settings[:stepSize] / 10.0^cycle
                    end

                elseif drivingCourse[end].s==CS.s_exit
                    break

                elseif drivingCourse[end].v==CS.v_target
                    break

                elseif drivingCourse[end].F_T <= drivingCourse[end].F_R
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                else
                    error("ERROR at acceleration until braking phase: With the step variable ",settings[:stepVariable]," the while loop will be left although v<v_target and s<s_exit in CS",CS.id,"  with s=" ,drivingCourse[end].s," m and v=",drivingCourse[end].v," m/s")
                end
                # delete last data point for recalculating the last step with reduced step size
                pop!(drivingCourse)
                pop!(accelerationSection.dataPoints)

            else # if the level of approximation is reached
                if drivingCourse[end].v<=0.0
                    # push!(accelerationSection.dataPoints, drivingCourse[end].i)
                    error("ERROR: The train stops during the acceleration phase in CS",CS.id," because the tractive effort is lower than the resistant forces.",
                    "       Before the stop the last point has the values s=",drivingCourse[end-1].s," m  v=",drivingCourse[end-1].v," m/s  a=",drivingCourse[end-1].a," m/s^2",
                    "       F_T=",drivingCourse[end-1].F_T," N  R_traction=",drivingCourse[end-1].R_traction," N  R_consist=",drivingCourse[end-1].R_consist," N  R_path=",drivingCourse[end-1].R_path," N.")

                elseif drivingCourse[end].v>CS.v_target
                    pop!(drivingCourse)
                    pop!(accelerationSection.dataPoints)
                elseif drivingCourse[end].s + s_braking > CS.s_exit
                    pop!(drivingCourse)
                    pop!(accelerationSection.dataPoints)

                elseif drivingCourse[end].F_T <= drivingCourse[end].F_R
                    (CS, drivingCourse)=addDiminishingPhase!(CS, drivingCourse, settings, train, CSs)
                    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, accelerationSection.type))

                else

                end
            end
        end #for

        if length(accelerationSection.dataPoints) > 1 # TODO: is it still possible that it is <=1 although there is a sepaate diminishing phase?
            # calculate the accumulated acceleration section information
            accelerationSection.v_exit=drivingCourse[end].v                               # exit speed (in m/s)
            accelerationSection.s_exit=drivingCourse[end].s                                # last position (in m)
            accelerationSection.length=accelerationSection.s_exit-accelerationSection.s_entry           # total length  (in m)
            accelerationSection.t=drivingCourse[end].t-drivingCourse[accelerationSection.dataPoints[1]].t       # total running time (in s)
            accelerationSection.E=drivingCourse[end].E-drivingCourse[accelerationSection.dataPoints[1]].E       # total energy consumption (in Ws)

            CS.v_target=max(drivingCourse[end].v, CS.v_entry)      # setting v_target to the last data points velocity which is the highest reachable value in this characteristic section or to v_entry in case it is higher when driving on a path with high resistances
            CS.t=CS.t+accelerationSection.t       # total running time (in s)
            CS.E=CS.E+accelerationSection.E       # total energy consumption (in Ws)

            merge!(CS.behaviorSections, Dict(:acceleration=>accelerationSection))
        end
    end # else: just return the given data point number without changes due to the acceleration phase
    return (CS, drivingCourse)
end #function addAccelerationPhaseUntilBraking!


## This function calculates the data points of the cruising phase.
#   Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for cruising if needed.
function addCruisingPhase!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, s_cruising::AbstractFloat, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection}, cruisingType::String)
    # traction effort and resisting forces (in N)
    calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "cruising") # TODO: or give cruisingSection.type instead of "cruising"?

    if drivingCourse[end].F_T < drivingCourse[end].F_R
        (CS, drivingCourse) = addDiminishingPhase!(CS, drivingCourse, settings, train, CSs)
        drivingCourse[end] = DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "cruising"))
        s_cruising = max(0.0, s_cruising-get(CS.behaviorSections, :diminishing, BehaviorSection()).length)
    end
    if drivingCourse[end].v>0.0 && drivingCourse[end].v<=CS.v_target && drivingCourse[end].s<CS.s_exit && drivingCourse[end].F_T >= drivingCourse[end].F_R
    # 11/22 old: if drivingCourse[end].v>0.0 && drivingCourse[end].v<=CS.v_target && drivingCourse[end].s<CS.s_exit
        cruisingSection=BehaviorSection()
        cruisingSection.type=cruisingType                                                       # type of behavior section
        cruisingSection.s_entry=drivingCourse[end].s                                            # first position (in m)
        # 11/22: now it is at the end of the BS: cruisingSection.s_exit=min(drivingCourse[end].s+s_cruising, CS.s_exit) # last position (in m)
        cruisingSection.v_entry=drivingCourse[end].v                                            # entry speed (in m/s)
        push!(cruisingSection.dataPoints, drivingCourse[end].i)

        # TODO: necessary?
        s_cruising=min(s_cruising, CS.s_exit-cruisingSection.s_entry)

        # traction effort and resisting forces (in N)
        calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "cruising") # TODO: or give cruisingSection.type instead of "cruising"?
        # 11/05 old: drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "cruising")) # TODO: or give cruisingSection.type instead of "cruising"?

        if settings[:massModel]=="homogeneous strip" && CS.id > 1
            currentStepSize=settings[:stepSize]
            for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
                while drivingCourse[end].s < CS.s_entry + train[:l_train] && drivingCourse[end].s<cruisingSection.s_entry+s_cruising && drivingCourse[end].F_T>=drivingCourse[end].F_R #&& drivingCourse[end].v<=CS.v_target && drivingCourse[end].s<CS.s_exit
                    # TODO: whithout CSs should work as well, no? while drivingCourse[end].s < CSs[CS.id].s_entry + train[:l_train] && drivingCourse[end].s<cruisingSection.s_entry+s_cruising && drivingCourse[end].F_T>=drivingCourse[end].F_R #&& drivingCourse[end].v<=CS.v_target && drivingCourse[end].s<CS.s_exit
                # the tractive effort is lower than the resisiting forces and the train has use the highest possible effort to try to stay at v_target OR the mass model homogeneous strip is used and parts of the train are still in former CS
                 #TODO: maybe just consider former CS with different path resistance?
                 #TODO: what about the case: After leaving a former CS with steep gradient the train can accelerate. Now in this tool the train will cruise at v_i. Just accelerating until v_target could make problems for energy saving by shortening the acceleration phase

                    # acceleration (in m/s^2):
                    drivingCourse[end].a=0.0

                    # create the next data point
                    if settings[:stepVariable]=="s in m"
                        push!(drivingCourse, moveAStep(drivingCourse[end], "s_cruising in m", currentStepSize, CS.id))
                    else
                        push!(drivingCourse, moveAStep(drivingCourse[end], "s_cruising in m", train[:l_train]/(10.0^cycle), CS.id)) # TODO which step size should be used?
                    end
                    push!(cruisingSection.dataPoints, drivingCourse[end].i)

                    # for testing
                    if drivingCourse[end].F_T > drivingCourse[end].F_R && drivingCourse[end].F_R > 0.0
                        error("In the cruising phase of CS",cruisingSection.id,": F_T=",drivingCourse[end].F_T," != F_R=",drivingCourse[end].F_R)
                    end

                    # traction effort and resisting forces (in N)
                    calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "cruising")
                end #while

                # check which limit was reached and adjust the currentStepSize for the next cycle
                if cycle < approximationLevel+1
                    if drivingCourse[end].s>cruisingSection.s_entry+s_cruising # TODO also the following? drivingCourse[end].s > CSs[CS.id].s_entry + train[:l_train]))
                        if settings[:stepVariable] == "s in m"
                            currentStepSize=cruisingSection.s_entry+s_cruising-drivingCourse[end-1].s
                        else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                        end
                    elseif drivingCourse[end].s==cruisingSection.s_entry+s_cruising # || drivingCourse[end].s==CS.s_exit
                        break
                    elseif drivingCourse[end].F_T < drivingCourse[end].F_R
                    #    if settings[:stepVariable] == "s in m"
                    #        currentStepSize=cruisingSection.s_entry+s_cruising-drivingCourse[end-1].s
                    #    else
                            currentStepSize = settings[:stepSize] / 10.0^cycle
                    #    end
                    elseif drivingCourse[end].s >= CS.s_entry + train[:l_train]
                    # TODO: whithout CSs should work as well, no? elseif drivingCourse[end].s >= CSs[CS.id].s_entry + train[:l_train]
                        break
                    else # TODO copied from addAccelerationPhase -> probably not needed here !?
                        error("ERROR at cruising phase: With the step variable ",settings[:stepVariable]," the while loop will be left although the if cases don't apply in CS",CS.id,"  with s=" ,drivingCourse[end].s," m and v=",drivingCourse[end].v," m/s")
                    end

                    # delete last data point for recalculating the last step with reduced step size
                    pop!(drivingCourse)
                    pop!(cruisingSection.dataPoints)

                else # if the level of approximation is reached
                    if drivingCourse[end].s>cruisingSection.s_entry+s_cruising
                        if cruisingSection.type == "clearing"
                        else
                            pop!(drivingCourse)
                            pop!(cruisingSection.dataPoints)
                        end
                    # 11/21 |->
                    elseif drivingCourse[end].s==cruisingSection.s_entry+s_cruising
                        break
                    # 11/21 ->|
                    elseif drivingCourse[end].F_T < drivingCourse[end].F_R

                        (CS, drivingCourse)=addDiminishingPhase!(CS, drivingCourse, settings, train, CSs)
                        drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "cruising"))

                    #    s_cruising=max(0.0, s_cruising-get(CS.behaviorSections, :diminishing, BehaviorSection()).length)

                    else

                    end
                end
            end #for
        end #if

        # TODO oder soll das lieber nach oben in den else des letzten Durchlaufs. Noch mal genauer ansehen, ob hier was doppelt gemoppelt ist
        # if drivingCourse[end].s<cruisingSection.s_entry+s_cruising
        if drivingCourse[end].s<cruisingSection.s_entry+s_cruising && drivingCourse[end].F_T >= drivingCourse[end].F_R
            drivingCourse[end].a=0.0    # acceleration (in m/s^2)

            # calculate the remaining cruising way
            s_cruisingRemaining=cruisingSection.s_entry+s_cruising-drivingCourse[end].s

            # create the next data point
            push!(drivingCourse, moveAStep(drivingCourse[end], "s_cruising in m", s_cruisingRemaining, CS.id))
            push!(cruisingSection.dataPoints, drivingCourse[end].i)
        end

        # calculate the accumulated cruising section information
        cruisingSection.v_exit=drivingCourse[end].v                                                       # exit speed (in m/s)
        cruisingSection.s_exit=drivingCourse[end].s                                                       # last position (in m)
        cruisingSection.length=cruisingSection.s_exit-cruisingSection.s_entry                             # total length  (in m)
        cruisingSection.t=drivingCourse[end].t-drivingCourse[cruisingSection.dataPoints[1]].t       # total running time (in s)
        cruisingSection.E=drivingCourse[end].E-drivingCourse[cruisingSection.dataPoints[1]].E       # total energy consumption (in Ws)

        CS.t=CS.t+cruisingSection.t       # total running time (in s)
        CS.E=CS.E+cruisingSection.E       # total energy consumption (in Ws)

        merge!(CS.behaviorSections, Dict(Symbol(cruisingSection.type) => cruisingSection))
    end # else: return the characteristic section without a cruising section

    return (CS, drivingCourse)
end #function addCruisingPhase!


## This function calculates the data points of the coasting phase.
# Therefore it gets its previous driving course and the characteristic section and returns the characteristic section and driving course including the coasting section
function addCoastingPhaseUntilBraking!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection})
    ## if the tail of the train is still located in a former characteristic section it has to be checked if its speed limit can be kept
    #formerSpeedLimits = detectFormerSpeedLimits(CSs, CS.id, drivingCourse[end], train[:l_train])

   if drivingCourse[end].v>CS.v_exit && drivingCourse[end].s<CS.s_exit
       coastingSection=BehaviorSection()
       coastingSection.type="coasting"             # type of behavior section
       coastingSection.s_entry=drivingCourse[end].s      # first position (in m)
       coastingSection.v_entry=drivingCourse[end].v      # entry speed (in m/s)
       push!(coastingSection.dataPoints, drivingCourse[end].i)

       currentStepSize=settings[:stepSize]  # initializing the step size that can be reduced near intersections
       # 08/24 old for cycle in 1:3                   # first cycle with normal step size, second cycle with reduced step size, third cycle with more reduced step size
       for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
            s_braking=ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel)
            while drivingCourse[end].v>CS.v_exit && drivingCourse[end].v<=CS.v_target && drivingCourse[end].s + s_braking < CS.s_exit # as long as s_i + s_braking < s_CSend
               # traction effort and resisting forces (in N):
               drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, coastingSection.type))

               # acceleration (in m/s^2):
               drivingCourse[end].a=(drivingCourse[end].F_T-drivingCourse[end].F_R)/train[:m_train]/train[:ξ_train]

               # creating the next data point
               push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS.id))
               push!(coastingSection.dataPoints, drivingCourse[end].i)

               s_braking=ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking])


            end # while

            # check which limit was reached and adjust the currentStepSize for the next cycle
            if cycle < approximationLevel+1
               if drivingCourse[end].s + s_braking > CS.s_exit
                   currentStepSize = settings[:stepSize] / 10.0^cycle

               elseif drivingCourse[end].v < CS.v_exit  # TODO: if accelereation and coasting functions will be combined this case is only for coasting
                    currentStepSize = settings[:stepSize] / 10.0^cycle

               elseif drivingCourse[end].v > CS.v_target
                   if settings[:stepVariable]=="v in m/s"
                       currentStepSize = CS.v_target-drivingCourse[end-1].v
                   else
                       currentStepSize = settings[:stepSize] / 10.0^cycle
                   end
               elseif drivingCourse[end].s + s_braking == CS.s_exit
                   break

               elseif drivingCourse[end].v == CS.v_exit
                   break

               else
                   # TODO: not needed. just for testing
                   error("ERROR at coasting until braking phase: With the step variable ",settings[:stepVariable]," the while loop will be left although v<v_target and s+s_braking<s_exit in CS",CS.id,"  with s=" ,drivingCourse[end].s," m and v=",drivingCourse[end].v," m/s")
               end
               # delete last data point for recalculating the last step with reduced step size
               pop!(drivingCourse)
               pop!(coastingSection.dataPoints)

           else # if the level of approximation is reached
                if drivingCourse[end].v<=0.0
                    println("INFO: The train stops during the coasting phase in CS",CS.id,"                                                                ",
                    "       Before the stop the last point has the values s=",drivingCourse[end-1].s," m  v=",drivingCourse[end-1].v," m/s  a=",drivingCourse[end-1].a," m/s^2",
                    "       F_T=",drivingCourse[end-1].F_T," N  R_traction=",drivingCourse[end-1].R_traction," N  R_consist=",drivingCourse[end-1].R_consist," N  R_path=",drivingCourse[end-1].R_path," N and s_braking=",s_braking,"m.")

               elseif drivingCourse[end].s + s_braking > CS.s_exit
                   # delete last data point because it went to far
                   pop!(drivingCourse)
                   pop!(coastingSection.dataPoints)

               elseif drivingCourse[end].v > CS.v_target # if the train gets to fast it has to brake  # TODO: if accelereation and coasting functions will be combined this case is different for coasting and also the order of if cases is different
                   # while coasting the train brakes to hold v_target (only one data point in the end of coasting is calculated like cruising at v_target)
                   drivingCourse[end-1].a=0.0
                   s_braking=ceil((CS.v_exit^2-drivingCourse[end-1].v^2)/2/train[:a_braking])

                   # recalculate s, t, v, E
                   #drivingCourse[end].Δs= CS.s_exit-drivingCourse[end-1].s - s_braking   # step size (in m)   # TODO: the coasting section is currently realised with using distance steps. For example t_braking could also be used
                   drivingCourse[end].Δs=min(currentStepSize, CS.s_exit-(drivingCourse[end-1].s+s_braking))       # TODO: if settings[:stepVariable]=="s in m"
                   drivingCourse[end].Δt=drivingCourse[end].Δs/drivingCourse[end-1].v                      # step size (in s)
                   drivingCourse[end].Δv=0.0                                                               # step size (in m/s)

                   drivingCourse[end].s=drivingCourse[end-1].s+drivingCourse[end].Δs                       # position (in m)
                   drivingCourse[end].t=drivingCourse[end-1].t+drivingCourse[end].Δt                       # point in time (in s)
                   drivingCourse[end].v=drivingCourse[end-1].v                                             # velocity (in m/s)

                   drivingCourse[end].ΔW=drivingCourse[end-1].F_T*drivingCourse[end].Δs                  # mechanical work in this step (in Ws)
                                       #   =0.0
                   drivingCourse[end].W=drivingCourse[end-1].W+drivingCourse[end].ΔW                 # mechanical work (in Ws)
                                       #   =drivingCourse[end-1].W
                   drivingCourse[end].ΔE=drivingCourse[end].ΔW                                           # energy consumption in this step (in Ws)
                                       #   =0.0
                   drivingCourse[end].E=drivingCourse[end-1].E+drivingCourse[end].ΔE                       # energy consumption (in Ws)
                                       #   =drivingCourse[end-1].E
               else

               end
           end
       end #for

       # calculate the accumulated coasting section information
       coastingSection.v_exit=drivingCourse[end].v                                                  # exit speed (in m/s)
       coastingSection.s_exit=drivingCourse[end].s                                                   # last position (in m)
       coastingSection.length=coastingSection.s_exit-coastingSection.s_entry                        # total length  (in m)
       coastingSection.t=drivingCourse[end].t-drivingCourse[coastingSection.dataPoints[1]].t   # total running time (in s)
       coastingSection.E=drivingCourse[end].E-drivingCourse[coastingSection.dataPoints[1]].E   # total energy consumption (in Ws)

       CS.t=CS.t+coastingSection.t          # total running time (in s)
       CS.E=CS.E+coastingSection.E          # total energy consumption (in Ws)

       merge!(CS.behaviorSections, Dict(:coasting=>coastingSection))
   end ## else: just return the given data point number without changes due to the coasting phase
   return (CS, drivingCourse)
end #function addCoastingPhaseUntilBraking!



## This function calculates the data points of the braking phase. (standard braking phase with only two data points)
#    Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingPhase!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection}) #, s_braking::AbstractFloat)
    # function addBrakingPhase!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, massModel::String, train::Dict, CSs::Vector{CharacteristicSection}) #, s_braking::AbstractFloat)
    if drivingCourse[end].v>CS.v_exit && drivingCourse[end].s<CS.s_exit
        brakingSection=BehaviorSection()
        brakingSection.type="braking"                           # type of behavior section
        brakingSection.s_entry=drivingCourse[end].s             # first position (in m)
        brakingSection.s_exit=CS.s_exit        # last position (in m)
        brakingSection.v_entry=drivingCourse[end].v             # entry speed (in m/s)
        push!(brakingSection.dataPoints, drivingCourse[end].i)   # refering from the breaking section to the first of its data points

        # traction effort and resisting forces (in N)
        drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, brakingSection.type))

        push!(drivingCourse, DataPoint())
        drivingCourse[end].i=drivingCourse[end-1].i+1                       # incrementing the number of the data point
        push!(brakingSection.dataPoints, drivingCourse[end].i)               # refering from the breaking section to the last of its data points

        # calculate s, t, v
        drivingCourse[end].s=brakingSection.s_exit                           # position (in m)
        drivingCourse[end].v=CS.v_exit                   # velocity (in m/s)
        drivingCourse[end].Δs=drivingCourse[end].s-drivingCourse[end-1].s   # step size (in m)
        drivingCourse[end].Δv=drivingCourse[end].v-drivingCourse[end-1].v   # step size (in m/s)

        # 09/21 old: rounding is not necessary.  drivingCourse[end-1].a=round((drivingCourse[end].v^2-drivingCourse[end-1].v^2)/2/drivingCourse[end].Δs, digits=approximationLevel)   # acceleration (in m/s^2) (rounding because it should not be less than a_braking)
        drivingCourse[end-1].a=(drivingCourse[end].v^2-drivingCourse[end-1].v^2)/2/drivingCourse[end].Δs
        #= if drivingCourse[end-1].a<train[:a_braking] || drivingCourse[end-1].a>=0.0
            println("")
            println("Warning: a_braking gets to high in CS ",CS.id, "   with a=",drivingCourse[end-1].a  ,"  >  ",train[:a_braking])
            println("  v=",drivingCourse[end].v,"   v_i-1=",drivingCourse[end-1].v, "   Δs=",drivingCourse[end].Δs)
            println("  v_exit=",CS.v_exit)
            println("")
        end =#
        drivingCourse[end].Δt=drivingCourse[end].Δv/drivingCourse[end-1].a          # step size (in s)
        drivingCourse[end].t=drivingCourse[end-1].t+drivingCourse[end].Δt           # point in time (in s)

        drivingCourse[end].ΔW=0.0                                                 # mechanical work in this step (in Ws)
        drivingCourse[end].W=drivingCourse[end-1].W+drivingCourse[end].ΔW     # mechanical work (in Ws)
        drivingCourse[end].ΔE=drivingCourse[end].ΔW                               # energy consumption in this step (in Ws)
        drivingCourse[end].E=drivingCourse[end-1].E+drivingCourse[end].ΔE           # energy consumption (in Ws)

        brakingSection.v_exit=drivingCourse[end].v                                  # exit speed (in m/s)
        brakingSection.length=drivingCourse[end].Δs                                # total length  (in m)
        brakingSection.t=drivingCourse[end].Δt                                # total running time (in s)
        brakingSection.E=drivingCourse[end].ΔE                                # total energy consumption (in Ws)

        CS.t=CS.t+brakingSection.t       # total running time (in s)
        CS.E=CS.E+brakingSection.E       # total energy consumption (in Ws)

        merge!(CS.behaviorSections, Dict(:braking=>brakingSection))
    end  # else: return the characteristic section without a braking section
    return (CS, drivingCourse)
end #function addBrakingPhase!



## This function calculates the data points of the braking phase. # 09/07 new braking phase with more than two data points
#    Therefore it gets its first data point and the characteristic section and returns the characteristic section including the behavior section for braking if needed.
function addBrakingPhaseStepwise!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection}) #, s_braking::AbstractFloat)
   if drivingCourse[end].v > CS.v_exit && drivingCourse[end].s < CS.s_exit
       brakingSection=BehaviorSection()
       brakingSection.type="braking"                           # type of behavior section
       brakingSection.s_entry=drivingCourse[end].s             # first position (in m)
       brakingSection.v_entry=drivingCourse[end].v             # entry speed (in m/s)
       push!(brakingSection.dataPoints, drivingCourse[end].i)   # refering from the breaking section to the first of its data points

       currentStepSize=settings[:stepSize]  # initializing the step size that can be reduced near intersections
       velocityIsPositive=true
       while drivingCourse[end].v > CS.v_exit && drivingCourse[end].s < CS.s_exit && velocityIsPositive
          # traction effort and resisting forces (in N):
          drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, brakingSection.type))

          # acceleration (in m/s^2):
          drivingCourse[end].a=train[:a_braking]

          # creating the next data point

          #TODO moveAStep should give back true or false for success or failure e.g. with dropping below v=0 m/s
           #at the moment it is only for stepVariable=="s in m"
          if settings[:stepVariable]=="s in m"
              if ((drivingCourse[end].v/drivingCourse[end].a)^2+2*currentStepSize/drivingCourse[end].a)<0.0 || (drivingCourse[end].v^2+2*currentStepSize*drivingCourse[end].a)<0.0
                  velocityIsPositive=false
                  break
              end
          end
          push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS.id))
          push!(brakingSection.dataPoints, drivingCourse[end].i)

          # s_braking=ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking])
       end # while

       if drivingCourse[end].v < CS.v_exit || !velocityIsPositive
           # calculate s, t, v
           drivingCourse[end].s=CS.s_exit                    # position (in m)
           drivingCourse[end].v=CS.v_exit                   # velocity (in m/s)
           drivingCourse[end].Δs=drivingCourse[end].s-drivingCourse[end-1].s   # step size (in m)
           drivingCourse[end].Δv=drivingCourse[end].v-drivingCourse[end-1].v   # step size (in m/s)

           #drivingCourse[end-1].a=round((drivingCourse[end].v^2-drivingCourse[end-1].v^2)/2/drivingCourse[end].Δs, digits=approximationLevel)   # acceleration (in m/s^2) (rounding because it should not be less than a_braking)
           drivingCourse[end-1].a=(drivingCourse[end].v^2-drivingCourse[end-1].v^2)/2/drivingCourse[end].Δs  # acceleration (in m/s^2)
        #   println("a_braking_last=",drivingCourse[end-1].a," m/s^2 und a_braking_standard=" , train[:a_braking])

       #    if drivingCourse[end-1].a<train[:a_braking] || drivingCourse[end-1].a>=0.0
       #        println("Warning: a_braking gets to high in CS ",CS.id, "   with a=",drivingCourse[end-1].a  ,"  >  ",train[:a_braking])
       #    end
           drivingCourse[end].Δt=drivingCourse[end].Δv/drivingCourse[end-1].a          # step size (in s)
           drivingCourse[end].t=drivingCourse[end-1].t+drivingCourse[end].Δt           # point in time (in s)

           drivingCourse[end].ΔW=0.0                                                 # mechanical work in this step (in Ws)
           drivingCourse[end].W=drivingCourse[end-1].W+drivingCourse[end].ΔW     # mechanical work (in Ws)
           drivingCourse[end].ΔE=drivingCourse[end].ΔW                               # energy consumption in this step (in Ws)
           drivingCourse[end].E=drivingCourse[end-1].E+drivingCourse[end].ΔE           # energy consumption (in Ws)
       elseif drivingCourse[end].s > CS.s_exit
          error("Beim Bremsen wurde das CS-Ende überschritten, aber nicht v_exit unterschritten !!")
       else

       end

       # calculate the accumulated coasting section information
       brakingSection.v_exit=drivingCourse[end].v                                                  # exit speed (in m/s)
       brakingSection.s_exit=drivingCourse[end].s                                                   # last position (in m)
       brakingSection.length=brakingSection.s_exit-brakingSection.s_entry                        # total length  (in m)
       brakingSection.t=drivingCourse[end].t-drivingCourse[brakingSection.dataPoints[1]].t   # total running time (in s)
       brakingSection.E=drivingCourse[end].E-drivingCourse[brakingSection.dataPoints[1]].E   # total energy consumption (in Ws)

       CS.t=CS.t+brakingSection.t       # total running time (in s)
       CS.E=CS.E+brakingSection.E       # total energy consumption (in Ws)

       merge!(CS.behaviorSections, Dict(:braking=>brakingSection))
   end  # else: return the characteristic section without a braking section
   return (CS, drivingCourse)
end #function addBrakingPhaseStepwise!


## This function calculates the data points for diminishing run when using maximum tractive effort and still getting slower
function addDiminishingPhase!(CS::CharacteristicSection, drivingCourse::Vector{DataPoint}, settings::Dict, train::Dict, CSs::Vector{CharacteristicSection})
    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, "diminishing"))

    if drivingCourse[end].F_T <= drivingCourse[end].F_R && drivingCourse[end].v > 0.0 && drivingCourse[end].s<CS.s_exit
        #drivingCourse[end].v>0.0 && drivingCourse[end].v<=CS.v_target && drivingCourse[end].s<CS.s_exit
        diminishingSection=BehaviorSection()
        diminishingSection.type="diminishing"                                                       # type of behavior section
        diminishingSection.s_entry=drivingCourse[end].s                                            # first position (in m)
        diminishingSection.v_entry=drivingCourse[end].v                                            # entry speed (in m/s)
        push!(diminishingSection.dataPoints, drivingCourse[end].i)


        currentStepSize=settings[:stepSize]   # initializing the step size that can be reduced near intersections
        for cycle in 1:approximationLevel+1   # first cycle with normal step size followed by cycles with reduced step size depending on the level of approximation
            s_braking=max(0.0, ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))
            while drivingCourse[end].F_T <= drivingCourse[end].F_R && drivingCourse[end].s+s_braking<CS.s_exit && drivingCourse[end].v>0.0       # as long as s_i + s_braking < s_CSend
            # 11/22 old without F_T<=F_R    while drivingCourse[end].s+s_braking<CS.s_exit && drivingCourse[end].v>0.0       # as long as s_i + s_braking < s_CSend

            #11/22    drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, diminishingSection.type))

                # acceleration (in m/s^2):
                drivingCourse[end].a=(drivingCourse[end].F_T-drivingCourse[end].F_R)/train[:m_train]/train[:ξ_train]
                # 11/21: old, only for cruising:
                #if drivingCourse[end].a==0.0
                #    error("ERROR: a=0 m/s^2 in the diminishing phase !   with  F_T=",drivingCourse[end].F_T,"  R_traction=",drivingCourse[end].R_traction,"  R_consist=",drivingCourse[end].R_consist,"  R_path=",drivingCourse[end].R_path)
                #end

                # create the next data point
                push!(drivingCourse, moveAStep(drivingCourse[end], settings[:stepVariable], currentStepSize, CS.id))
                push!(diminishingSection.dataPoints, drivingCourse[end].i)

                s_braking=max(0.0, ceil((CS.v_exit^2-drivingCourse[end].v^2)/2/train[:a_braking], digits=approximationLevel))
                drivingCourse[end]=DataPoint(calculateForces!(drivingCourse[end], train, settings[:massModel],  CSs, diminishingSection.type))
            end #while

            # check which limit was reached and adjust the currentStepSize for the next cycle
            if cycle < approximationLevel+1
                if drivingCourse[end].v<=0.0
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                elseif drivingCourse[end].s + s_braking > CS.s_exit
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                elseif drivingCourse[end].s + s_braking==CS.s_exit
                # 11/21 old without s_braking: elseif drivingCourse[end].s==CS.s_exit
                    break

                elseif drivingCourse[end].F_T > drivingCourse[end].F_R
                    currentStepSize = settings[:stepSize] / 10.0^cycle

                else
                    error("ERROR during diminishing run: With the step variable ",settings[:stepVariable]," the while loop will be left although s+s_braking<s_exit && v>0.0  in CS",CS.id,"  with s=" ,drivingCourse[end].s," m and v=",drivingCourse[end].v," m/s")
                end
                # delete last data point for recalculating the last step with reduced step size
                pop!(drivingCourse)
                pop!(diminishingSection.dataPoints)

            else # if the level of approximation is reached
                if drivingCourse[end].v<=0.0
                    # push!(diminishingSection.dataPoints, drivingCourse[end].i)
                    error("ERROR: The train stops during diminishing run in CS",CS.id," because the maximum tractive effort is lower than the resistant forces.",
                    "       Before the stop the last point has the values s=",drivingCourse[end-1].s," m  v=",drivingCourse[end-1].v," m/s  a=",drivingCourse[end-1].a," m/s^2",
                    "       F_T=",drivingCourse[end-1].F_T," N  R_traction=",drivingCourse[end-1].R_traction," N  R_consist=",drivingCourse[end-1].R_consist," N  R_path=",drivingCourse[end-1].R_path," N.")

                elseif drivingCourse[end].s + s_braking > CS.s_exit
                    pop!(drivingCourse)
                    pop!(diminishingSection.dataPoints)

                elseif drivingCourse[end].F_T > drivingCourse[end].F_R
                    break

                else

                end
            end
        end #for

        if length(diminishingSection.dataPoints) > 1 # TODO: necessary? May it be possible that there is no diminishing because braking has to start
            # calculate the accumulated diminishing section information
            diminishingSection.v_exit=drivingCourse[end].v                                                     # exit speed (in m/s)
            diminishingSection.s_exit=drivingCourse[end].s                                                      # last position (in m)
            diminishingSection.length=diminishingSection.s_exit-diminishingSection.s_entry                       # total length  (in m)
            diminishingSection.t=drivingCourse[end].t-drivingCourse[diminishingSection.dataPoints[1]].t   # total running time (in s)
            diminishingSection.E=drivingCourse[end].E-drivingCourse[diminishingSection.dataPoints[1]].E   # total energy consumption (in Ws)
            CS.t=CS.t+diminishingSection.t             # total running time (in s)
            CS.E=CS.E+diminishingSection.E             # total energy consumption (in Ws)

            merge!(CS.behaviorSections, Dict(:diminishing=>diminishingSection))
        end
    end

    return (CS, drivingCourse)
end #function addDiminishingPhase!

end #module MovingPhases
