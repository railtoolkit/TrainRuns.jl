# TrainRuns

[![License: ISC](https://img.shields.io/badge/license-ISC-green.svg)](https://opensource.org/licenses/ISC) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.6448563.svg)](https://doi.org/10.5281/zenodo.6448563) [![Build Status](https://github.com/railtoolkit/TrainRuns.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/railtoolkit/TrainRuns.jl/actions/workflows/CI.yml?query=branch%3Amain)

------------

## About

TrainRuns.jl is a step towards open science and open data in railway engineering. Its modular design offers the possibility to serve as a basis for future optimization and development. TrainRuns.jl is suitable for qualitative calculations to compare different trains, and it is publicly available, and we invite others to collaborate.

------------

## Installation

Use the package manager provided by [julia](https://julialang.org):

```julia
julia> # use the ] key
pkg> add TrainRuns
pkg> # use backspace
julia> using TrainRuns
```

The required julia packages are

- YAML.jl
- JSONSchema.jl
- DataFrames.jl

------------

## Minimal working example

```julia
using TrainRuns

train = Train("train.yaml") # load train from file
path  = Path("path.yaml")   # load running path from file

runtime = trainrun(train, path)[end,:t]

println("The train needs $runtime seconds for the running path.")
```

------------

## Further Information

Visit the repository [TrainRuns.jl-Tutorials](https://github.com/railtoolkit/TrainRuns.jl-Tutorials) for tutorials in either Jupyther Notebooks or Pluto Notebooks. There you can find, for instance, a [basic tutorial](https://github.com/railtoolkit/TrainRuns.jl-Tutorials/blob/main/basic.ipynb).

Please refer to the automated [documentation](https://www.railtoolkit.org/TrainRuns.jl/) for technical details of the used functions.

------------

## Acknowledgement

This work was supervised by South Westphalia University of Applied Sciences and Technical University Braunschweig.

------------

## License

[![Open Source Initiative Approved License logo](https://opensource.org/files/OSIApproved_100X125.png "Open Source Initiative Approved License logo")](https://opensource.org)

ISC License (ISC)

Copyright 2022 Max Kannenberg, Martin Scheidt

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
