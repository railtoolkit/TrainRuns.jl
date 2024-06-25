"""
    df_2_vector(df::DataFrame)::Vector{Dict}
    
Convert DataFrame to a Vector of Dicts

# Example
```julia-repl
julia> TrainRuns.df_2_vector(DataFrame(ages=[12,20]))
2-element Vector{Dict}:
 Dict(:ages => 12)
 Dict(:ages => 20)
```
"""
function df_2_vector(df::DataFrame)::Vector{Dict}
    vec = Dict[]

    for row in eachrow(df)
        dict = Dict(Symbol(cn) => row[cn] for cn in names(df))
        push!(vec, dict)
    end

    return vec
end

"""
    value_changes(field::AbstractVector)::Vector{Bool}

Return vector if value changes compared to the one before.
The first element is alwas true.

# Example
```julia-repl
julia> TrainRuns.value_changes(DataFrame(ages=[12,20]))
5-element Vector{Bool}:
 1
 0
 1
 0
```
"""
function value_changes(field::AbstractVector)::Vector{Bool}
    if (isempty(field))
        return Vector{Bool}[]
    end

    value = first(field)
    keep::Vector{Bool} = [true]

    for current_value in field[2:end]
        if value != current_value
            value = current_value
            push!(keep, true)
        else
            push!(keep, false)
        end
    end

    return keep
end
