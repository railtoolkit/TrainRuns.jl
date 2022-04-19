# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Categories: Added, Changed, Deprecated, Removed, Fixed, and Security.

## [Unreleased]


## Version [0.8] 2022-01-20

### Changed

Refactor the modular structure:
* Divide TrainRun into TrainRunCalc with the main functions and Import for importing data from yaml files
* Extract the modules Export and AdditionalOutput from TrainRunCalc
* Divide the module Operationsmodes and add its functions to TrainRunCalc and EnergySaving
* Add the remaining functions of the module types to EnergySaving
* Divide the module MovingPhases into Behavior and DrivingDynamics
* Rename the module Preparation to Characteristics


## Version [0.7] 2022-01-14

### Changed

Refactor all mutable structs as a Dictionaries:
* Refactor the mutable struct EnergySavingModification from types.jl as a Dictionary in OperationsModes.jl
* Refactor the mutable struct CharacteristicSection from types.jl as a Dictionary in Preparation.jl
* Refactor the mutable struct BehaviorSection from types.jl as a Dictionary in MovingPhases.jl
* Refactor the mutable struct DataPoint from types.jl as a Dictionary in MovingPhases.jl
* Remove behavior section "cruisingAfterCoasting"
* Rename some variables


## Version [0.6.2] 2021-12-17

### Added

Add function addStandstill! for creating the BehaviorSection standstill:
* Add function addStandstill! to MovinPhases.jl
* Use function addStandstill! in OperationModes.jl
* Rename the BehaviorSection standStill to standstill

### Fixed

* Fix: Rename addStartingPhase! to addBreakFreePhase!


## Version [0.6.1]

### Added

Add an attribute to DataPoint to record the corresponding driving behavior
* Add the attribute behavior to Datapoint in types.jl
* Attach the corresponding behavior to data points in MovingPhases.jl
* Attach the behavior "standStill" to the last data point of the driving course in OperationModes.jl

### Changed

* Rework Output.jl for outputting the data points' behavior


## Version [0.6]

### Changed

Refactor some of the mutable structs from types.jl as Dictionaries
* Remove the mutable structs Train, Path, PathSection, Settings and MovingSection
* Create Dictionaries for train, path an settings in Input.jl
* Create a Dictionary for the whole moving section in Preperation.jl and a function for copying the moving section in OperationModes.jl
* Change the type of existing Dictionary keys from String to Symbol


## Version [0.5.3]

### Changed

Rename variables in every .jl an .yaml file


## Version [0.5.2]

Merge fixing branches


## Version [0.5.1]

### Changed

Rename the real world path file


## Version [0.5]

### Changed

Refactor modules for diminishing run and tractive effort velocity pairs
* Add the seperate moving phase "diminishing run" for steep ascents where a train runs with maximum tractive effort while the driving resistances are even higher
* Refactor tractiveEffortArray to tractiveEffortVelocityPairs
* Rename file path and folder path to directory


## Version [0.4.1]

### Changed

Rename waypoints
* rename "waypoints" to "dataPoints" and "Waypoint" to "DataPoint"


## Version [0.4]

### Changed

Refactor and fix modules EnergySaving, OperationModes and MovingPhases
* add the general used level of accuracy from v0.3 to EnergySaving and OperationModes
* fix OperationModes and MovingPhases for steep ascents where a train runs with maximum tractive effort while the driving resistances are even higher


## Version [0.3]

### Changed

Refactor module MovingPhases
* extract repeatedly occuring code lines and create smaller functions (e.g. the function moveAStep)
* integrate a new approach for calculating the waypoints near intersections (e.g. including an editable level of accuracy)


## Version [0.2]

### Changed

Modules and variables were renamed.


## Version [0.1] 2021-02-19

Proof of concept and master thesis submission.


[Unreleased]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.8...master
[0.8]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.7...v0.8
[0.7]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.6.2...v0.7
[0.6.2]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.6.1...v0.6.2
[0.6.1]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.6...v0.6.1
[0.6]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.5.3...v0.6
[0.5.3]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.5.2...v0.5.3
[0.5.2]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.5.1...v0.5.2
[0.5.1]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.5...v0.5.1
[0.5]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.4.1...v0.5
[0.4.1]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.4...v0.4.1
[0.4]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.3...v0.4
[0.3]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.2...v0.3
[0.2]: https://github.com/railtoolkit/TrainRun.jl/compare/v0.1...v0.2
[0.1]: https://github.com/railtoolkit/TrainRun.jl/releases/tag/v0.1