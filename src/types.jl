module types

export copyCharacteristicSection, copyBehaviorSection

#@enum behavior breakFree=1 clearing=2 acceleration=3 cruising=4 diminishing=6 coasting=7 braking=8 standstill=9

## different sections the whole path can be devided in the following
## smallest section of the path is the behavior section. It relates to the containing data points via their identifier.
function copyBehaviorSection(original::Dict)
    bsDataPoints=[]
    for i in 1:length(original[:dataPoints])
        push!(bsDataPoints, original[:dataPoints][i])
    end
    copiedBS = Dict(#:type => behavior,                 # type of behavior section: breakFree, clearing, acceleration, cruising, diminishing, coasting, braking or standstill
                :type => original[:type],           # type of behavior section: "breakFree", "clearing", "acceleration", "cruising", "diminishing", "coasting", "braking" or "standstill"
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

function copyCharacteristicSection(originalCS::Dict)
    allBs=[:breakFree, :clearing, :acceleration, :cruising, :diminishing, :coasting, :braking, :standstill]
    copiedBSs = Dict()
    for bs in 1: length(allBs)
        if haskey(originalCS[:behaviorSections], allBs[bs])
            merge!(copiedBSs, Dict(allBs[bs] => originalCS[:behaviorSections][allBs[bs]]))
        end #if
    end #for

    copiedCS=Dict(:id => originalCS[:id],               # identifier
                 :s_entry => originalCS[:s_entry],      # first position (in m)
                 :s_exit => originalCS[:s_exit],        # last position  (in m)
                 :length => originalCS[:length],        # total length  (in m)
                 :r_path => originalCS[:r_path],        # path resistance (in ‰)
                #      :behaviorSections => copy(originalCS[:behaviorSections]),  # list of containing behavior sections
                 :behaviorSections => copiedBSs,  # list of containing behavior sections
                 :t => originalCS[:t],                  # total running time (in s)
                 :E => originalCS[:E],                  # total energy consumption (in Ws)
                 :v_limit => originalCS[:v_limit],      # speed limit (in m/s)
                 :v_peak => originalCS[:v_peak],        # maximum reachable speed (in m/s)
                 :v_entry => originalCS[:v_entry],      # maximum entry speed (in m/s)
                 :v_exit => originalCS[:v_exit])        # maximum exit speed (in m/s)

        return copiedCS
end # CharacteristicSection

end #module
