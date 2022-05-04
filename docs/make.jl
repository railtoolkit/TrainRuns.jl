using TrainRuns
using Documenter

DocMeta.setdocmeta!(TrainRuns, :DocTestSetup, :(using TrainRuns); recursive=true)

makedocs(;
    modules=[TrainRuns],
    authors="Max Kannenberg, Martin Scheidt, and contributors",
    repo="https://github.com/railtoolkit/TrainRuns.jl/blob/{commit}{path}#{line}",
    sitename="TrainRuns.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://railtoolkit.github.io/TrainRuns.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/railtoolkit/TrainRuns.jl",
    devbranch="development",
)
