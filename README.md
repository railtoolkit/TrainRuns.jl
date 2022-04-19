# TrainRun

[![License: ISC](https://img.shields.io/badge/license-ISC-green.svg)](https://opensource.org/licenses/ISC) 

------------

# About

TrainRun.jl is a step towards open science and open data in railway engineering. Its modular design offers the possibility to serve as a basis for future optimization and development. TrainRun.jl is suitable for qualitative calculations to compare different trains, and it is publicly available, and we invite others to collaborate.

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

train_directory =  "data/trains/train_freight_V90withOreConsist.yaml"
running_path_directory = "data/paths/path_1_10km_nConst_vConst.yaml"
settings_directory = "data/settings.yaml"
(train, running_path, settings) = importYamlFiles(train_directory, running_path_directory, setting_directory)

train_run = calculateDrivingDynamics(train, running_path, settings)
```

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
