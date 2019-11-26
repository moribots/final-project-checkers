INSTRUCTIONS:

**SETUP**

Fork this repository.

Use `wstool set` with the SSH to YOUR repository, NOT the group one to automatically set the origin master.

Then, do `git remote add upstream git@github.com:ME495-EmbeddedSystems/final-project-checkers.git`.

Now you can push to upstream using `git push upstream` or your own repo using `git push origin` or both using `git push --all`.

We should discuss workflow conventions before proceeding.

**WORKFLOW**

Work on your own branch and push to `origin` whenever you want to back up your progress.

When you are ready to push to `upstream` (the shared repo):

* `pull` master branch from `upstream`
* `merge` master branch INTO your personal branch 
* Make sure code works and fix any bugs
* Once everything works, merge your personal branch back INTO master
* `push` to `upstream` (and to origin if you want)