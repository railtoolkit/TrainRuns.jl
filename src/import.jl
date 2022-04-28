#!/usr/bin/env julia
# -*- coding: UTF-8 -*-
# __julia-version__ = 1.7.2
# __author__        = "Max Kannenberg"
# __copyright__     = "2020-2022"
# __license__       = "ISC"

"""
Read the input information from YAML files for train, path and settings, save it in different dictionaries and return them.
"""
function importYamlFiles(trainDirectory::String, pathDirectory::String)
    train = importFromYaml(:train, trainDirectory)
    path = importFromYaml(:path, pathDirectory)

    return (train, path)
end #function importYamlFiles

 """
 Read the train information from a YAML file, save it in a Dict and return it.
 """
function importFromYaml(dataType::Symbol, directory::String)
    dataSet = String(dataType)
    data = YAML.load(open(directory))
    if collect(keys(data))[1] != dataSet
        error("ERROR at reading the ", dataSet, " yaml file: The data set is called ", collect(keys(data))[1]," and not ", dataSet, ".")
    end
    dataKeys = collect(keys(data[dataSet]))
    dataKeys = collect(keys(data[dataSet]))
    dataValues = collect(values(data[dataSet]))
    dictionary = Dict()
    for number in 1:length(dataKeys)
        merge!(dictionary, Dict(Symbol(dataKeys[number]) => dataValues[number]))
    end
    return dictionary
end # function importFromYaml
