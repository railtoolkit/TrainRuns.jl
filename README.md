# TrainRun

------------

# Installation

The required julia packages are
   - YAML.jl
   - Dates.jl
   - DataFrames.jl
   - CSV.jl
   - Plots.jl

Review the settings.yaml file for your appropriate settings.

------------

# Minimal working example

```julia
include("../src/TrainRun.jl")
using .TrainRun

train =  "data/trains/train_freight_V90withOreConsist.yaml"
running_path = "data/paths/path_1_10km_nConst_vConst.yaml"
settings = "data/settings.yaml"

train_run = calculateDrivingDynamics(train, running_path, settings)
```
------------

# History

## Version 0.5.2
Merge fixing branches

## Version 0.5.1
Rename the real world path file

## Version 0.5

Refactor modules for diminishing run and tractive effort velocity pairs

- Add the seperate moving phase "diminishing run" for steep ascents where a train runs with maximum tractive effort while the driving resistances are even higher
- Refactor tractiveEffortArray to tractiveEffortVelocityPairs
- Rename file path and folder path to directory


## Version 0.4.1

Rename waypoints

- rename "waypoints" to "dataPoints" and "Waypoint" to "DataPoint"


## Version 0.4

Refactor and fix modules EnergySaving, OperationModes and MovingPhases

- add the general used level of accuracy from v0.3 to EnergySaving and OperationModes
- fix OperationModes and MovingPhases for steep ascents where a train runs with maximum tractive effort while the driving resistances are even higher


## Version 0.3

Refactor module MovingPhases

- extract repeatedly occuring code lines and create smaller functions (e.g. the function moveAStep)
- integrate a new approach for calculating the waypoints near intersections (e.g. including an editable level of accuracy)


## Version 0.2

Modules and variables were renamed.


## Version 0.1

Proof of concept and master thesis submission.

------------

# Acknowledgement

This work was supervised by South Westphalia University of Applied Sciences and Technical University Braunschweig.

------------

# License

  [![Open Source Initiative Approved License logo](https://opensource.org/files/OSIApproved_100X125.png "Open Source Initiative Approved License logo")](https://opensource.org)

ISC License (ISC)

Copyright 2021 Max Kannenberg

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
