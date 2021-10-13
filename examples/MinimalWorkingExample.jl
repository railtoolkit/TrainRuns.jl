include("../src/RailwayDrivingDynamics.jl")
using .RailwayDrivingDynamics

# println("")
# println("________________________")
# println("")

allPaths=["../data/paths/path_1_10km_nConst_vConst.yaml" , "../data/paths/path_2_10km_nVar_vConst.yaml", "../data/paths/path_3_10km_nConst_vVar.yaml", "../data/paths/path_4_real_Ostsachsen_DG-DN_spp_5.yaml"]

settingsFilePath="../data/settings.yaml"

for pathFilePath in allPaths
    # println("")
    # println(" -    -    -     -     -     -      -     -    -")
    # println("path: ", pathFilePath)

    vehicleFilePath="../data/vehicles/vehicle_passenger_intercity.yaml"
    testDict1=calculateDrivingDynamics(vehicleFilePath, pathFilePath, settingsFilePath)

    sleep(2)
    vehicleFilePath="../data/vehicles/vehicle_passenger_suburban.yaml"
    testDict2=calculateDrivingDynamics(vehicleFilePath, pathFilePath, settingsFilePath)


    sleep(2)
    vehicleFilePath="../data/vehicles/vehicle_freight.yaml"
    testDict3=calculateDrivingDynamics(vehicleFilePath, pathFilePath, settingsFilePath)


    # println("")
    # println("")
    # println("")
    # println("")
end
