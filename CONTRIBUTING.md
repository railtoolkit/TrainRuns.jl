# Contributing

When contributing to this repository, please first discuss the change you wish to make via issue,
email, or any other method with the owners of this repository before making a change. 

Please note we have a code of conduct, please follow it in all your interactions with the project.

## Julia Development Environment

Link your local git repository to Julia:
```console
$ ln -s ~/path/to/TrainRuns.jl ~/.julia/dev/TrainRuns 
```

Have a look how to develop Julia packages: https://github.com/ShozenD/julia-pkg-dev
You might want to use `Revise.jl` as well:
```julia
Pkg.add("Revise")
```
and then just load with `using Revise` (preferably by putting it in the `~/.julia/config/startup.jl` file).

You can overide the standard TrainRuns package with the local development branch (see linking above) with:
```julia
julia> # use the ] key
(@v1.x) pkg> develop TrainRuns
(@v1.x) pkg> # use backspace
julia> using TrainRuns # local development branch will be loaded
```

If you want to add a dependency use:
```julia
julia> # use the ] key
(@v1.x) pkg> activate TrainRuns
(TrainRuns) pkg>
```

## Files in TrainRuns

| file            | concern                        |
| --------------- | ------------------------------ |
| TrainRuns.jl    | main file and function         |
| types.jl        | special TrainRuns types        |
| constructors.jl | type constructors              |
| formulary.jl    | formulars from literature      |
| output.jl       | transformation into DataFrames |

## Reporting Issues

* It's always good to start with a quick search for an existing issue to post on,
  or related issues for context, before opening a new issue
* Including minimal examples is greatly appreciated
* If it's a bug, or unexpected behaviour, reproducing on the latest development version
  (`Pkg.add(name="TrainRuns", rev="main")`) is a good gut check and can streamline the process,
  along with including the first two lines of output from `versioninfo()`

## Style Guidelines

TODO

## Git Recommendations For Pull Requests

* Avoid working from the `main` branch of your fork, creating a new branch will make it
  easier if TrainRuns.jl `main` branch changes and you need to update your pull request;
* All PRs and issues should be opened against the `main` branch not against the current release;
* Run tests of your code before sending any commit to GitHub. Only push changes when 
  the tests of the change are passing locally. In particular note that it is not a problem
  if you send several commits in one push command to GitHub as CI will be run only once then;
* If any conflicts arise due to changes in TrainRuns.jl `main` branch, prefer updating your pull
  request branch with `git rebase` (rather than `git merge`), since the latter will introduce a merge 
  commit that might confuse GitHub when displaying the diff of your PR, which makes your changes more 
  difficult to review. Alternatively use conflict resolution tool available at GitHub;
* Please try to use descriptive commit messages to simplify the review process;
* Using `git add -p` or `git add -i` can be useful to avoid accidently committing unrelated changes;
* Maintainers get notified of all changes made on GitHub. However, what is useful is writing a short
  message after a sequence of changes is made summarizing what has changed and that the PR is ready
  for a review;
* When linking to specific lines of code in discussion of an issue or pull request, hit the `y` key
  while viewing code on GitHub to reload the page with a URL that includes the specific commit that 
  you're viewing. That way any lines of code that you refer to will still be correct in the future, even 
  if additional commits are pushed to the branch you are reviewing;
* Please make sure you follow the code formatting guidelines when submitting a PR;
  Also preferably do not modify parts of code that you are not editing as this makes
  reviewing the PR harder (it is better to open a separate maintenance PR
  if e.g. code layout can be improved);
* If a PR is not finished yet and should not be reviewed yet then it should be opened as DRAFT 
  (in this way maintainers will know that they can ignore such PR until it is made non-draft or the author
  asks for a review).

## Contribution Ideas

Models for:
* Breakaway (currently simple like acceleration)
* braking (currently constant braking with a certain value)
* inhomogeneous mass band (consider non-uniform mass distribution (cf. Wende, 2003 p. 96f.))

More exact calculation of resistances for:
* the track (e.g. including curves, switches, tunnels) 
* for the train (e.g. there are vehicle resistance equations especially for high-speed trains, which are not yet considered in the tool (cf. Wende, 2003 p. 152 f.))

Input/calculation of tractive force (currently only tractive force-speed pairs. This could be extended to include tractive force functions that apply to specific velocity ranges (cf. Brünger, et al., 2014 p. 69).

Calculation of energy, cf. (Wende, 2003 p. 324).

switching between different step variables in one train run or implementation of variable step size (currently it is necessary to choose between s, t and v and to specify a step size that becomes smaller at intersections, but is otherwise constant)

Driver behaviour, in order to be able to determine the driving behavior of the vehicles more realistically.

Energy-saving driving.

good plots would be nice

Pluto notebook show case
