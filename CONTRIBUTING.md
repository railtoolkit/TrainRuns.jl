# Contributing

When contributing to this repository, please first discuss the change you wish to make via issue,
email, or any other method with the owners of this repository before making a change. 

Please note we have a code of conduct, please follow it in all your interactions with the project.

## Julia Development Environment

Link your local git repository to Julia:
```console
$ ln -s ~/path/to/TrainRun.jl ~/.julia/dev/TrainRun 
```

Have a look how to develop Julia packages: https://github.com/ShozenD/julia-pkg-dev
You might want to use `Revise.jl` as well:
```julia
Pkg.add("Revise")
```
and then just load with `using Revise` (preferably by putting it in the `~/.julia/config/startup.jl` file).

You can overide the standard TrainRun package with the local development branch (see linking above) with:
```julia
julia> # use the ] key
(@v1.x) pkg> develop TrainRun
(@v1.x) pkg> # use backspace
julia> using TrainRun # local development branch will be loaded
```

If you want to add a dependency use:
```julia
julia> # use the ] key
(@v1.x) pkg> activate TrainRun
(TrainRun) pkg>
```

## TrainRun files

TODO!

## Reporting Issues

* It's always good to start with a quick search for an existing issue to post on,
  or related issues for context, before opening a new issue
* Including minimal examples is greatly appreciated
* If it's a bug, or unexpected behaviour, reproducing on the latest development version
  (`Pkg.add(name="TrainRun", rev="master")`) is a good gut check and can streamline the process,
  along with including the first two lines of output from `versioninfo()`

## Style Guidelines

TODO

## Git Recommendations For Pull Requests

* Avoid working from the `master` branch of your fork, creating a new branch will make it
  easier if TrainRun.jl `master` branch changes and you need to update your pull request;
* All PRs and issues should be opened against the `master` branch not against the current release;
* Run tests of your code before sending any commit to GitHub. Only push changes when 
  the tests of the change are passing locally. In particular note that it is not a problem
  if you send several commits in one push command to GitHub as CI will be run only once then;
* If any conflicts arise due to changes in TrainRun.jl `master` branch, prefer updating your pull
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
