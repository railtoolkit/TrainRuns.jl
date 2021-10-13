# Ausführen im pkg mode mit >>>>> ] test RailwayDrivingDynamics <<<<

using RailwayDrivingDynamics, Test


# das hier waren die zu testenden Funktionen:
#foo(x::T, y::T) where T <: Real = x + y - 5
#bar(z::Float64) = foo(sqrt(z), z)
#=
 That file may typically contain calls to other
 test files via include(fname). In our case there
 is only two functions to test, so we will just
 write in runtests.jl directly:
=#

# todo: Beispiel:
#=
@testset "foo" begin
    x, y = 5, 7
    @test foo(x, y) == 7
    x = "blah"
    @test_throws MethodError foo(x, y)
end

@testset "bar" begin
    z = 4.
    @test bar(z) == 1.
end =#



    #vehicleFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Fahrzeuge\\vehicle_passenger_IC2.yaml"
    #vehicleFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Fahrzeuge\\vehicle_passenger_SiemensDesiroClassic.yaml"
    vehicleFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Fahrzeuge\\vehicle_freight_V90withOreConsist.yaml"

    #pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Strecken\\path_10km_noGradient.yaml"
    pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Strecken\\path_10km_differentGradient.yaml"
    #pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Strecken\\path_10km_different_v_limits.yaml"
    #pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Fahrzeug- und Streckendaten\\Strecken\\path_real_Ostsachsen_V220_DG-DN_spp_5.yaml"

    settingsFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\settings.yaml"
    #settingsFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\settings_s.yaml"

    testDict=calculateDrivingDynamics(vehicleFilePath, pathFilePath, settingsFilePath)

    println("Vergleichswerte für FBS berechnet")

    #=

    #pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\track_DAFW_DAFN.yaml"
    #pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\test.yaml"
    settingsFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\settings.yaml"
    testDict=calculateDrivingDynamics(vehicleFilePath, pathFilePath, settingsFilePath)

    vehicleFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\vehicle.yaml"
    pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\track_DAFW_DAFN.yaml"
    #pathFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\path_Heilbronn.yaml"

    settingsFilePath="C:\\Users\\Offline-Konto\\Dateien\\Masterarbeit\\TU-BS_IfEV\\Julia-Files\\settings.yaml"

    calculateDrivingDynamics(vehicleFilePath, pathFilePath, settingsFilePath)

    =#
println("test finished")



# toto: einfach so ausführen geht nicht.
#       da muss ich wahrscheilich noch anders importieren oder auf das Modul verweisen?
# Ausführen kann ich es aber auch mit>>>>> test RailwayDrivingDynamics <<<< im pkg mode. Das geht.
