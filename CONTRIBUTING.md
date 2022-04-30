# Contributing

When contributing to this repository, please first discuss the change you wish to make via issue,
email, or any other method with the owners of this repository before making a change. 

Please note we have a code of conduct, please follow it in all your interactions with the project.

# Julia Development Environment

Link your local git repository to Julia:
```console
$ ln -s ~/path/to/TrainRun.jl ~/.julia/dev/TrainRun 
```

Have a look here how to develop Julia packages: https://github.com/ShozenD/julia-pkg-dev
You might want to use `Revise.jl` as well:
```julia
Pkg.add("Revise")
```
and then just load with `using Revise` (preferably by putting it in the `~/.julia/config/startup.jl` file)

```julia
(@v1.x) pkg> develop TrainRun
(@v1.x) pkg> activate TrainRun
(TrainRun) pkg>
```


# Pull Request Process

  * add your changes to the CHANGELOG.md under [Unreleased]
  * TODO!