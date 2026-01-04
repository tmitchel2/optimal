---
argument-hint: [create-unit-test-issues-for-least-covered-files]
description: Create unit testing issues for the least covered files
---

# Create Unit Test Issues For Least Covered Files

- Ensure that you are on the main branch and have latest source checked.
- Run ./utils/get-least-covered-files.sh 5
- For each file reported you should use github-issue-create subagent to create a unit testing issue.
