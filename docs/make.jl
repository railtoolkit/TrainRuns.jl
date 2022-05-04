using TrainRun
using Documenter

DocMeta.setdocmeta!(TrainRun, :DocTestSetup, :(using TrainRun); recursive=true)

makedocs(;
    modules=[TrainRun],
    authors="Max Kannenberg, Martin Scheidt, and contributors",
    repo="https://github.com/railtoolkit/TrainRun.jl/blob/{commit}{path}#{line}",
    sitename="TrainRun.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://railtoolkit.github.io/TrainRun.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/railtoolkit/TrainRun.jl",
    devbranch="development",
)
