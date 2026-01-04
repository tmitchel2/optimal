---
name: using-git-worktrees
description: You are an expert user of git worktrees.  You interact with git worktrees to implement tasks.
---

# Using Git Worktrees Skill

This skill guides interaction with git worktrees, you will you the git cli for working with worktrees.

## Reference help for 'git worktree' command

```bash
git worktree add [-f] [--detach] [--checkout] [--lock [--reason <string>]]
                        [-b <new-branch>] <path> [<commit-ish>]
   or: git worktree list [-v | --porcelain [-z]]
   or: git worktree lock [--reason <string>] <worktree>
   or: git worktree move <worktree> <new-path>
   or: git worktree prune [-n] [-v] [--expire <expire>]
   or: git worktree remove [-f] <worktree>
   or: git worktree repair [<path>...]
   or: git worktree unlock <worktree>
```

## Using Git Worktrees Notes

- Executing scripts in a git worktree may require one "cd" before calling the script and then another "cd" after to return to the original directory. E.g. cd /private/tmp/claude/graphlessdb-issue-164 && git pull origin main && cd /users/blah/github/graphlessdb.  Put another way you will need to cd into the worktree folder before each execution of a script and then you will need to cd back to the original folder to reset the pwd back to what it was before.
- Check for existing branches before creating one. Use `git worktree add <path> <existing-branch>` if branch exists, not -b flag
- Create new git worktrees under the folder /tmp/claude/.
