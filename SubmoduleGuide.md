# Getting Started with Submodules

## What is a submodule

<p> A git submodule is a speciific folder or adress within a repository or porject that directs the contents of another respository into the specified file location. Submodules are overlooked by the git machine unless the submodule is entered. When looking at changes throught github desktop, changes made inside the module will not be detected and must be handled through the console.<p>

<p>The main component behind a git submodule is the `.gitmodules` file that is can either be auto-generated or manually created. The contents of this file provide the necessary information sumodule setup in the project, but further steps are required to setup code usage.<p>

The contents of the `.gitmodules`:
```
[submodule "libs/3128-common"]
	path = libs/3128-common
	url = https://github.com/Team3128/3128-common.git
```
<br>

## Setting up a Submodule

Submodules can be accessed and configured using the `git submodule` commands in the terminal.


### Removing Submodule and Directories
To remove submodule, directory, and related chache:
```
git rm [-f | --force] [-n] [-r] [--cached] [--ignore-unmatch] [--quiet] [--pathspec-from-file=<file> [--pathspec-file-nul]] [--] [<pathspec>…​]
```

```
git rm -r --cached libs/3128-common
```

### Adding Submodules
To add the submodule from the github cloud to the `libs/3128-common` destination:
```
git submodule [--quiet] add [-b "branch"] [-f|--force] [--name "name"] [--reference "repository"] [--] "repository" ["path"]
```

```
git submodule add https://github.com/Team3128/3128-common.git libs/3128-common
```
`--force` may be appended if required.

### Configuring Gradle Implementation
In order for porjects in the main repository to acess files from the submodules, the directory must be included in the gradle proejct, and it must be configured as an implementation. <br>

Navigate to the `settings.gradle` file in the main proejct and append the following inclusion and project:
```
include ':libs:3128-common'
project(':libs:3128-common').projectDir = file('libs/3128-common')
```
<br>

Navigate to the `build.gradle` file in the main project and add the project as a dependency:
```
dependencies {

    implementation project(':libs:3128-common')

	// other dependencies
}
```
<br>

Then `cd` into the submodules and build the gradle project. Starting from the main proejct:
```
cd libs/3128-common
./gradlew build
```
<br>

After, `cd` back to the main project and run a gralde build:
```
cd ../..
./gradlew build
```


### Quick for Submodule Steps
1. Remove any existing directories or chache
```
git rm -r --cached libs/3128-common
```
2. Clone from default branch or specified branch: 
```
git submodule add https://github.com/Team3128/3128-common.git libs/3128-common
```
3. Include the project by navigating to `settings.gradle` and adding: 
```
include ':libs:3128-common'
project(':libs:3128-common').projectDir = file('libs/3128-common')
```
4. Add the dependency by navigating to `build.gralde` and adding:
```
dependencies {

    implementation project(':libs:3128-common')

	// other dependencies
}
```
5. Build the submodule project:
```
cd libs/3128-common
./gradlew build
```
6. Build the main project:
```
cd ../..
./gradlew build
```

### Basic Git Commands
The following is taken from the git help page.

> To use these commands on a submodule, the `git submodule foreach` can be appended to the front to call for all modules, or the command can be called by navigating to the submodule directory.

To start a working area:
* `git clone`		Clone a repository into a new directory
* `git init`		Create an empty Git repository or reinitialize an existing one

To work on the current change
* `git add`			Add file contents to the index
* `git mv`			Move or rename a file, a directory, or a symlink
* `git restore`		Restory working tree files
* `git rm`			Remove files from the working tree and from the index

To examine the history and state:
* `git status`		Show the working tree status
* `git diff`		Show changes between commits, commit and working tree, ect.
* `git bisect`		Use binary search to fine the commit that introduced a bug
* `git grep`		Print lines matching a pattern
* `git log`			Show commit logs
* `git show`		Show various types of objects

To grow, mark, and tweak your common history:
* `git branch` 		List, create, or delete branches
* `git checkout`	Switch branches or restore working tree files
* `git commit`		Record changes to the repository
* `git merge`		Join two or more development histories together
* `git rebase`		Reapply commits on top of another base tip
* `git reset`		Reset current HEAD to the specified state
* `git revert`		Revert some existing commits
* `git stash`		Stach the changes in a dirty working directory away
* `git switch`		Switch branches
* `git submodule`	Initialize, update, or inspect submodules
* `git tag`			Create, list, delete, or verify a tag object signed with GPG

To collaborate:
* `git fetch`		Downlead objects and refs from another repository
* `git pull`		Fetch from and integrate with another repository or local branch
* `git push` 		Update remote refs along with associated objects

<br> Call `git help -a` to read all git commands.