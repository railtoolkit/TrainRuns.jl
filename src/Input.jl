# TODO: >>if train[:type] == freight::trainType<< does not work only in checkTrainType(train::Dict). because ::Main.TrainRun.Import.trainType != ::Main.TrainRun.Input.trainType    -> why? checkTrainType ist therefore deactivated.
# TODO: adapt function informAboutUnusedKeys for Input.jl
module Input

export checkAndSetInput!

@enum trainType passenger=1 freight=2 motorCoachTrain=3


approximationLevel = 6  # value for approximation to intersections TODO further explanation

"""
Read the input information from YAML files for train, path and settings, save it in different dictionaries and return them.
"""
function checkAndSetInput!(train::Dict, path::Dict, settings::Dict)
     checkAndSetTrain!(train)
     checkAndSetPath!(path)
     checkAndSetSettings!(settings)

     if settings[:detailOfOutput] == "points of interest" && !haskey(path, :pointsOfInterest)
         settings[:detailOfOutput] = "driving course"
         println("INFO at checking the input for settings and path: settings[:detailOfOutput] is 'points of interest' but the path does not have a list for pointsOfInterest. Therefore the detailOfOut is changed to 'driving course'.")
     end
     return (train, path, settings)
end #function checkAndSetInput!

"""
Read the train information from a YAML file, save it in a train Dict and return it.
"""
function checkAndSetTrain!(train::Dict)
 # check train information from input dictionary

    checkString(train, "train", :name)    # train's name
    # TODO checkId ? train[:id]                                        # train's identifier
    checkTrainType(train)                  # passenger or freight or motorCoachTrain

    checkPositiveNumber(train, "train", :length, "m")           # total length (in m)

    checkAndSetPositiveNumber!(train, "train", :v_limit, "m/s", 1000.0/3.6)   # train's speed limit (in m/s)
    checkAndSet_a_braking!(train)                          # a_braking

    checkPositiveNumber(train, "train", :m_td, "kg")            # mass on the traction unit's driving axles (in kg)
    checkAndSetPositiveNumber!(train, "train", :m_tc, "kg", 0.0)     # mass on the traction unit's carrying axles (in kg)

    checkAndSetSum!(train, "train", :m_t, :m_td, :m_tc)         # mass of the traction unit (in kg)
    checkAndSetPositiveNumber!(train, "train", :m_w, "kg", 0.0)      # mass of the set of wagons (consist)  (in kg)
    checkAndSetSum!(train, "train", :m_train, :m_t, :m_w)       # total mass (in kg)

    checkAndSetRotationMassFactors!(train)
    checkAndSetTractiveEffortVelocityPairs!(train)  # pairs of velocity and tractive effort

    # coefficients for the vehicle resistance of the traction unit
    checkAndSetRealNumber!(train, "train", :Δv_t, "m/s")            # coefficient for velocitiy difference between traction unit and outdoor air (in m/s)
    checkAndSetPositiveNumber!(train, "train", :f_Rtd0, "‰", 0.0)   # coefficient for basic resistance due to the traction units driving axles (in ‰)
    checkAndSetPositiveNumber!(train, "train", :f_Rtc0, "‰", 0.0)   # coefficient for basic resistance due to the traction units carring axles (in ‰)
    checkAndSetPositiveNumber!(train, "train", :F_Rt2, "N", 0.0)    # coefficient for air resistance of the traction units (in N)

    # coefficients for the vehicle resistance of the set of wagons (consist)
    checkAndSetRealNumber!(train, "train", :Δv_w, "m/s")            # coefficient for velocitiy difference between set of wagons (consist) and outdoor air (in m/s)
    checkAndSetPositiveNumber!(train, "train", :f_Rw0, "‰", 0.0)    # coefficient for basic resistance of the set of wagons (consist)  (in ‰)
    checkAndSetPositiveNumber!(train, "train", :f_Rw1, "‰", 0.0)    # coefficient for the consists resistance to rolling (in ‰)
    checkAndSetPositiveNumber!(train, "train", :f_Rw2, "‰", 0.0)    # coefficient fo the consistsr air resistance (in ‰)

#    informAboutUnusedKeys(train, "train") # inform the user, which Symbols of the input dictionary are not used in this tool

    return train
end #function checkAndSetTrain!

function checkAndSetPath!(path::Dict)
 # check path information from input dictionary

    checkString(path, "path", :name)
    # TODO checkId ? path[:id]                                                  # path identifier
    checkAndSetSections!(path)
    checkAndSetPOIs!(path)

# TODO:    informAboutUnusedKeys(path, "path") # inform the user, which Symbols of the input dictionary are not used in this tool

    return path
end # function checkAndSetPath!


## settings for the calculation
function checkAndSetSettings!(settings::Dict)
 # check settings information from input dictionary

    checkString(settings, "settings", :massModel, ["mass point", "homogeneous strip"])      # model type of the train's mass "mass point" or "homogeneous strip"
    checkString(settings, "settings", :stepVariable, ["s in m", "t in s", "v in m/s"])      # step variable of the step method "s in m", "t in s" or "v in m/s"
    checkPositiveNumber(settings, "settings", :stepSize, "("*settings[:stepVariable]*")")  # step size (unit depends on stepVariable: s in m, t in s and v in m/s)
    checkAndSetBool!(settings, "settings", :operationModeMinimumRunningTime)                # operation mode "minimum running time"
    checkAndSetBool!(settings, "settings", :operationModeMinimumEnergyConsumption)          # operation mode "minimum energy consumption"
    checkString(settings, "settings", :typeOfOutput, ["julia dictionary", "CSV"])           # output as "julia dictionary" or as "CSV"

    if settings[:typeOfOutput] == "CSV"
        checkString(settings, "settings", :csvDirectory)
        # TODO: it could be checked if the path is existing on the pc
    end # if

    checkString(settings, "settings", :detailOfOutput, ["minimal", "points of interest", "driving course"])    # should the output be "minimal" or are "points of interest" or the complete "driving course" required?

# TODO:    informAboutUnusedKeys(settings, "settings")         # inform the user, which Symbols of the input dictionary are not used in this tool

    return settings
end # function checkAndSetSettings!

function checkAndSetBool!(dictionary::Dict, dictionaryType::String, key::Symbol)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key])!=Bool
            error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of the Symbol :",String(key)," is not correct. The value has to be of type Bool.")
        end
    else
        merge!(dictionary, Dict(key => false))
        println("WARNING at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(key)," or its value is missing. Therefore ",String(key),"=",dictionary[key]," is assumed and used.")
    end
    return dictionary
end #function checkAndSetBool!

function checkPositiveNumber(dictionary::Dict, dictionaryType::String, key::Symbol, unit::String)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key]) <: Real && dictionary[key] >= 0.0
            return true
        else
            error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(key)," is no real floating point number >=0.0.")
        end
    else
        error("ERROR at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(key)," is missing. It has to be added with a value of type real floating point number >0.0.")
    end
    return false
end #function checkPositiveNumber

function checkAndSetPositiveNumber!(dictionary::Dict, dictionaryType::String, key::Symbol, unit::String, default::Real)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key]) <: Real && dictionary[key] >= 0.0
        else
            error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(key)," is no real floating point number >=0.0.")
        end
    else
        merge!(dictionary, Dict(key => default))
        println("WARNING at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(key)," is missing. Therefore ",String(key),"=",default," ",unit," will be assumed and used." )
    end

    return dictionary
end #function checkAndSetPositiveNumber!

function checkAndSetRealNumber!(dictionary::Dict, dictionaryType::String, key::Symbol, unit::String)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        if typeof(dictionary[key]) <: Real
        else
            error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(key)," is no real number.")
        end
    else
        if key == :Δv_t
            value = 15.0/3.6
        elseif key == :Δv_w
            if dictionary[:type] == passenger::trainType || dictionary[:type] == motorCoachTrain::trainType
                value = 15.0/3.6
            elseif dictionary[:type] == freight::trainType
                value = 0.0
            end # if
        else
            value = 0.0
        end
        merge!(dictionary, Dict(key => value))
        println("WARNING at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(key)," is missing. Therefore ",String(key),"=",value," ",unit," will be assumed and used." )
    end

    return dictionary
end #function checkAndSetRealNumber!

function checkString(dictionary::Dict, dictionaryType::String, key::Symbol, validValues::Vector{String})
    # TODO change getString! to getSymbol! or to getEnum! ?
    if haskey(dictionary,key) && dictionary[key]!=nothing
        value = dictionary[key]
        if typeof(value)==String
            for validValue in validValues
                if value == validValue
                    return true
                end
            end
        end
        error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(key)," is wrong. It has to be one of the following String values: ", validValues)
    else
        error("ERROR at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(key)," is missing. It has to be added with one of the following String values: ", validValues)
    end
    return false
end #function checkString
# second method of function checkString without validValues
function checkString(dictionary::Dict, dictionaryType::String, key::Symbol)
    if haskey(dictionary,key) && dictionary[key]!=nothing
        value = dictionary[key]
        if typeof(value)==String
            return true
        end
        error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(key)," is wrong. It has to be of type String.")
    else
        error("ERROR at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(key)," is missing. It has to be added.")
    end
    return false
end #function checkString

function checkTrainType(train::Dict)
    if haskey(train,:type) && train[:type]!=nothing
#=    #    println("train[:type]=",train[:type],"    typeof(train[:type])=", typeof(train[:type]))
        if train[:type] == passenger::trainType || train[:type] == freight::trainType || train[:type] == motorCoachTrain::trainType

            return true
        else
            error("ERROR at checking the input dictionary for the train: The value of train[:type] is wrong. It has to be passenger, freight or motorCoachTrain of enum type trainType.")
        end =#
    else
        error("ERROR at checking the input dictionary for the train: The Symbol :type is missing. It has to be added with the value passenger, freight or motorCoachTrain of enum type trainType.")
    end
    return false
end # function checkTrainType

function checkAndSet_a_braking!(train::Dict)
    if haskey(train, :a_braking) && train[:a_braking]!=nothing
        if typeof(train[:a_braking]) <: Real
            return train
        else
            error("ERROR at checking the input dictionary for the train: The value of the a_braking is no real floating point number <0.0.")
        end

        if train[:a_braking] > 0.0
            train[:a_braking] =-train[:a_braking]
            println("WARNING at checking the input dictionary for the train: The value for a_braking is >0.0. The braking acceleration has to be <0.0. Therefore a_braking=",train[:a_braking]," m/s^2 is used." )
        elseif train[:a_braking] == 0.0
            error("ERROR at checking the input dictionary for the train: The value for a_braking is 0.0. The braking acceleration has to be <0.0.")
        end
    else
        error("ERROR at checking the input dictionary for the train: The Symbol :a_braking is missing. It has to be added with a value of type real floating point number <0.0.")
    end

    return train
end #function checkAndSet_a_braking!

function checkAndSetSum!(dictionary::Dict, dictionaryType::String, sum::Symbol, summand1::Symbol, summand2::Symbol)
    if haskey(dictionary,sum) && dictionary[sum]!=nothing
        if typeof(dictionary[sum]) <: Real && dictionary[sum] >= 0.0
            difference = abs(dictionary[sum] - (dictionary[summand1]+dictionary[summand2]))
            if difference > 1/(10^approximationLevel)
                error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(sum)," is not exactly the sum of ",String(summand1)," and ",String(summand2),". It differs by ",difference,".")
            end
        else
            error("ERROR at checking the input dictionary for the ",dictionaryType,": The value of ",String(sum)," is no real floating point number >=0.0.")
        end
    else
        merge!(dictionary, Dict(sum => dictionary[summand1]+dictionary[summand2]))
        println("WARNING at checking the input dictionary for the ",dictionaryType,": The Symbol :",String(sum)," is missing. Therefore ",String(sum)," = ",String(summand1)," + ",String(summand2)," = ",dictionary[sum]," was calculated and will be used." )
    end

    return dictionary
end #function checkAndSetSum!

function checkAndSetRotationMassFactors!(train::Dict)
    if haskey(train, :ξ_train) && train[:ξ_train]!=nothing && typeof(train[:ξ_train]) <: Real
        if train[:ξ_train]>0.0
            if haskey(train, :ξ_t) && train[:ξ_t]!=nothing && train[:ξ_t]>0.0  && typeof(train[:ξ_t]) <: Real && (train[:m_w]==0.0 || (haskey(train, :ξ_w) && train[:ξ_w]!=nothing && typeof(train[:ξ_w]) <: Real))
        ##        println("train[:ξ_train]=",train[:ξ_train],"   Rest=",(train[:ξ_t]*train[:m_t] + train[:ξ_w]*train[:m_w])/train[:m_train])
        #        println("train[:ξ_t]=",train[:ξ_t]," train[:m_t]=",train[:m_t],"  train[:ξ_w]=",train[:ξ_w]," train[:m_w]=",train[:m_w],"  train[:m_train]=",train[:m_train])

        #        println("absDiff=", abs(train[:ξ_train] - (train[:ξ_t]*train[:m_t] + train[:ξ_w]*train[:m_w])/train[:m_train]))
                difference = abs(train[:ξ_train] - (train[:ξ_t]*train[:m_t] + train[:ξ_w]*train[:m_w])/train[:m_train])
                if difference > 1/(10^approximationLevel)
                    error("ERROR at checking the input dictionary for the train: The value of ξ_train is not exactly ξ_train=(ξ_t*m_t + ξ_w*m_w)/m_train. It differs by ",difference,".")
                end
            end
        else
            error("ERROR at checking the input dictionary for the train: The value of :ξ_train is no real floating point number >0.0.")
        end
    elseif haskey(train, :ξ_t) && train[:ξ_t]!=nothing  && typeof(train[:ξ_t]) <: Real && (train[:m_w]==0.0 || (haskey(train, :ξ_w) && train[:ξ_w]!=nothing && typeof(train[:ξ_w]) <: Real))
        if train[:ξ_t]>0.0
            ξ_t=train[:ξ_t]
        else
            error("ERROR at checking the input dictionary for the train: The value of :ξ_t is no real floating point number >0.0.")
        end
        if train[:m_w]>0.0
            if train[:ξ_w]>=0.0
                ξ_w=train[:ξ_w]
            else
                error("ERROR at checking the input dictionary for the train: The value of :ξ_w is no real floating point number >=0.0.")
            end
        else
            ξ_w=0.0
        end

        ξ_train=(ξ_t*train[:m_t] + ξ_w*train[:m_w])/train[:m_train]  # rotation mass factor of the whole train (without unit)
        merge!(train, Dict(:ξ_train => ξ_train))
    else
        error("ERROR at checking the input dictionary for the train: The Symbols :ξ_train or :ξ_t and :ξ_w are missing. They has to be added with a value of type real floating point number.")
    end

    return train
end #function checkAndSetRotationMassFactors!

function checkAndSetTractiveEffortVelocityPairs!(train::Dict)  # pairs of velocity and tractive effort
    if haskey(train,:tractiveEffortVelocityPairs) && train[:tractiveEffortVelocityPairs]!=nothing
        pairs = train[:tractiveEffortVelocityPairs]

        # check if the elements of the array have the correct type
        errorDetected=false

        for row in 1:length(pairs)
            if typeof(pairs[row][1]) <: Real && pairs[row][1]>=0.0
            else
                errorDetected=true
                println("ERROR at checking the input dictionary for the train: The speed value of train[:tractiveEffortVelocityPairs] in row ", row ," is no real floating point number >=0.0.")
            end
            if typeof(pairs[row][2]) <: Real && pairs[row][2]>=0.0
            else
                errorDetected=true
                println("ERROR at checking the input dictionary for the train: The tractive effort value of train[:tractiveEffortVelocityPairs] in row ", row ," is no real floating point number >=0.0.")
            end

            if row>=2 && pairs[row][1] <= pairs[row-1][1]
                errorDetected=true
                println("ERROR at checking the input dictionary for the train: The speed value of train[:tractiveEffortVelocityPairs] in row ", row ," (v=",pairs[row][1]," m/s) is not higher than the speed value in the previous row (v=",pairs[row-1][1]," m/s).")
            end
        end # for
        if errorDetected
            error("ERROR at checking the input dictionary for the train: Only real floating point number >=0.0 are allowed for speed and tractive effort. The speed values have to be listed from low to high.")
        end

        # create tractiveEffortVelocityPairs
        if pairs[1][1]>0.0    # if there is no F_T for v=0.0, the first known value is used
            newPairs=[]
            push!(newPairs, [0.0, pairs[1][2]])
            println("WARNING at checking the input dictionary for the train: The tractive effort for v=0.0 m/s is missing. Therefore the first given value F_T(v=",pairs[1][1]," m/s)=",pairs[1][2]," N will be used." )
            for row in 1:length(pairs)
                push!(newPairs, [pairs[row][1], pairs[row][2]])
            end # for
            merge!(train, Dict(:tractiveEffortVelocityPairs => newPairs))
        end

        if length(pairs[1])>2
            println("INFO according the train dictionary: Only the first two columns of train[:tractiveEffortVelocityPairs] are used in this tool.")
        end

    else
        error("ERROR at checking the input dictionary for the train: There has to be the Symbol :tractiveEffortVelocityPairs filled with a list of pairs of velocity and tractive effort.")
    end # if

    return train
end #function checkAndSetTractiveEffortVelocityPairs!

function checkAndSetSections!(path::Dict)
    # check the section information
    if haskey(path,:sections) && path[:sections]!=nothing
        sections = path[:sections]

        checkedSections = []
        increasing = false
        decreasing = false

 #TODO: throw error for each issue or collect the issues and use the Bool errorDetected like in checkTractiveEffortVelocityPairs!?
        # check values for section==1
        checkAndSetRealNumber!(sections[1], "path[:sections][1]", :s_start, "m")                      # first point of the section (in m)
        checkAndSetRealNumber!(sections[1], "path[:sections][1]", :s_end, "m")                        # first point of the next section (in m)
        checkAndSetPositiveNumber!(sections[1], "path[:sections][1]", :v_limit, "m/s", 1000.0/3.6)    # paths speed limt (in m/s)
        checkAndSetRealNumber!(sections[1], "path[:sections][1]", :f_Rp, "‰")                # specific path resistance of the section (in ‰)

        push!(checkedSections, sections[1])

        if sections[1][:s_start] < sections[1][:s_end]
            increasing = true
        elseif sections[1][:s_start] > sections[1][:s_end]
            decreasing = true
        else
            pop!(checkedSections)
            println("WARNING at checking the input dictionary for the path: The first section of :sections has the same position for starting and end point. The section will be deleted and not used in the tool.")
        end


        for section in 2:length(sections)
            checkAndSetRealNumber!(sections[section], "path[:sections]["*string(section)*"]", :s_start, "m")                      # first point of the section (in m)
            checkAndSetRealNumber!(sections[section], "path[:sections]["*string(section)*"]", :s_end, "m")                        # first point of the next section (in m)
            checkAndSetPositiveNumber!(sections[section], "path[:sections]["*string(section)*"]", :v_limit, "m/s", 1000.0/3.6)    # paths speed limt (in m/s)
            checkAndSetRealNumber!(sections[section], "path[:sections]["*string(section)*"]", :f_Rp, "‰")                # specific path resistance of the section (in ‰)

            push!(checkedSections, sections[section])

            # compare the section's start and end position
            if sections[section][:s_start] < sections[section][:s_end]
                increasing = true
            elseif sections[section][:s_start] > sections[section][:s_end]
                decreasing = true
            else
                pop!(checkedSections)
                println("WARNING at checking the input dictionary for the path: The ",section,". section of :sections has the same position for starting and end point. The section will be deleted and not used in the tool.")
            end
            if increasing && decreasing
                error("ERROR at checking the input dictionary for the path: The positions of the :sections are not increasing/decreasing consistently. The direction in the ",section,". section differs from the previous.")
            end


            if length(checkedSections)>1 && sections[section][:s_start] != checkedSections[end-1][:s_end]
                error("ERROR at checking the input dictionary for the path[:sections]: The starting position of the ",section,". section does not euqaul the last position of the previous section. The sections have to be sequential.")
                # TODO: maybe if there is a gab create a new section and only if there a jumps in the wrong direction throw an error?
            end
        end #for
    else
        error("ERROR at checking the input dictionary for the path: The Symbol :sections is missing. It has to be added with a list of sections. Each has to be a dictionary with the keys :s_tart, :s_end, :v_limit and :f_Rp.")
    end

    return path
end #function checkAndSetSections!

function checkAndSetPOIs!(path::Dict)
    # read the section starting positions and corresponding information
    if haskey(path,:pointsOfInterest)
        if path[:pointsOfInterest] != nothing
            pointsOfInterest = path[:pointsOfInterest]

            sortingNeeded = false
            errorDetected = false
            for element in 1:length(pointsOfInterest)
                if typeof(pointsOfInterest[element]) <: Real
                    if element > 1
                        if pointsOfInterest[element] < pointsOfInterest[element-1]
                            sortingNeeded = true
                            println("INFO at checking the input dictionary for the path: The point of interest in element ", element ," (",pointsOfInterest[element]," m) has to be higher than the value of the previous element (",pointsOfInterest[element-1]," m). The points of interest will be sorted.")
                        end
                    end
                else
                    errorDetected = true
                    println("ERROR at checking the input dictionary for the path: The point of interest in element ", element ," is no real floating point number.")
                end
            end # for

            if errorDetected
                error("ERROR at checking the input dictionary for the path: The values of pointsOfInterest have to be corrected.")
            end
            if sortingNeeded == true
                sort!(pointsOfInterest)
            end

            copiedPOIs = []
            for element in 1:length(pointsOfInterest)
                if element == 1
                    push!(copiedPOIs, pointsOfInterest[element])
                elseif element > 1 && pointsOfInterest[element] > pointsOfInterest[element-1]
                    push!(copiedPOIs, pointsOfInterest[element])
                end
            end # for
            path[:pointsOfInterest] = copiedPOIs

        else
            println("INFO at checking the input dictionary for the path: The key pointsOfInterest exists but without values.")
            delete!(path, :pointsOfInterest)
        end
    end

    return path
end #function checkAndSetPOIs!

function informAboutUnusedKeys(dictionary::Dict, dictionaryType::String)         # inform the user which Symbols of the input dictionary are not used in this tool
#=    if length(dictionary)>0
        println("INFO at checking the input dictionary for the ",dictionaryType,": The following Keywords are not used in this tool:")
        for key in keys(dictionary)
            println("  - ",key)
        end
    end =#
end #function informAboutUnusedKeys

end # module Input
