# TrainRuns

 [![License: ISC][license-img]][license-url] [![DOI][zenodo-img]][zenodo-url] [![Build Status][ci-img]][ci-url] [![SQAaaS badge][SQAaaS-img]][SQAaaS-url] [![All Contributors][Contributors-img]][Contributors-url]

------------

## About

TrainRuns.jl is a step towards open science and open data in railway engineering. Its modular design offers the possibility to serve as a basis for future optimization and development. TrainRuns.jl is suitable for qualitative calculations to compare different trains, and it is publicly available, and we invite others to collaborate.

------------

## Features

TODO

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

## Contributors

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/MaxKannenberg"><img src="https://avatars.githubusercontent.com/u/95709892?v=4?s=100" width="100px;" alt="Max Kannenberg"/><br /><sub><b>Max Kannenberg</b></sub></a><br /><a href="#research-MaxKannenberg" title="Research">🔬</a> <a href="#code-MaxKannenberg" title="Code">💻</a> <a href="#ideas-MaxKannenberg" title="Ideas, Planning, & Feedback">🤔</a> <a href="#maintenance-MaxKannenberg" title="Maintenance">🚧</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/kaat0"><img src="https://avatars.githubusercontent.com/u/142348?v=4?s=100" width="100px;" alt="Martin Scheidt"/><br /><sub><b>Martin Scheidt</b></sub></a><br /><a href="#mentoring-kaat0" title="Mentoring">🧑‍🏫</a> <a href="#tutorial-kaat0" title="Tutorials">✅</a> <a href="#code-kaat0" title="Code">💻</a> <a href="#test-kaat0" title="Tests">⚠️</a> <a href="#tool-kaat0" title="Tools">🔧</a> <a href="#research-kaat0" title="Research">🔬</a> <a href="#data-kaat0" title="Data">🔣</a> <a href="#maintenance-kaat0" title="Maintenance">🚧</a> <a href="#ideas-kaat0" title="Ideas, Planning, & Feedback">🤔</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

------------

## License

[![Open Source Initiative Approved License logo](https://149753425.v2.pressablecdn.com/wp-content/uploads/2009/06/OSIApproved_100X125.png "Open Source Initiative Approved License logo")](https://opensource.org)

ISC License (ISC)

Copyright 2022 Max Kannenberg, Martin Scheidt

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

[license-img]: https://img.shields.io/badge/license-ISC-green.svg
[license-url]: https://opensource.org/licenses/ISC

[ci-img]: https://github.com/railtoolkit/TrainRuns.jl/actions/workflows/CI.yml/badge.svg?branch=main
[ci-url]: https://github.com/railtoolkit/TrainRuns.jl/actions/workflows/CI.yml?query=branch%3Amain

[zenodo-img]: https://zenodo.org/badge/DOI/10.5281/zenodo.6448563.svg
[zenodo-url]: https://doi.org/10.5281/zenodo.6448563

[SQAaaS-img]: https://img.shields.io/badge/sqaaas%20software-silver-lightgrey
[SQAaaS-url]: https://api.eu.badgr.io/public/assertions/qXPM75OnSLmDxbEQlcuzxw "SQAaaS silver badge achieved"

[Contributors-img]: https://img.shields.io/github/all-contributors/railtoolkit/TrainRuns.jl?color=ee8449&style=flat-square
[Contributors-url]: #Contributors
