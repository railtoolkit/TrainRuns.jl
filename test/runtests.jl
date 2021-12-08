# access in pkg mode with >>>>> ] test TrainRun <<<<

using TrainRun, Test


allPaths=[]
push!(allPaths, "../data/paths/path_1_10km_nConst_vConst.yaml")
push!(allPaths, "../data/paths/path_2_10km_nVar_vConst.yaml")
push!(allPaths, "../data/paths/path_3_10km_nConst_vVar.yaml")
push!(allPaths, "../data/paths/path_4_real_Ostsachsen_DG-DN_spp_5.yaml")

allSettings=[]
push!(allSettings, "../data/settings.yaml")

allTrains=[]
push!(allTrains, "../data/trains/train_freight_V90withOreConsist.yaml")
push!(allTrains, "../data/trains/train_yaml_files\\train_passenger_SiemensDesiroClassic.yaml")
push!(allTrains, "../data/trains/train_passenger_IC2.yaml")

for pathDirectory in allPaths
    # println("")
    # println(" -    -    -     -     -     -      -     -    -")
    # println("path: ", pathDirectory)
   for trainDirectory in allTrains
       # println("train: ", trainDirectory)
       for settingsDirectory in allSettings
           testDict=calculateDrivingDynamics(trainDirectory, pathDirectory, settingsDirectory)

           sleep(2)

           # println("")
           # println("")
           # println("")
       end
   end
end

println("test finished")
