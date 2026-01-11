#!/usr/bin/env bash
set -e

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Find an issue with "todo" label (or any OPEN issue if no todo-labeled issues exist)
ISSUE_NUMBER=$(gh issue list --label "todo" --state open --json number --jq '.[0].number' 2>/dev/null)

if [ -z "$ISSUE_NUMBER" ] || [ "$ISSUE_NUMBER" = "null" ]; then
    # Fallback: get first open issue without "in-progress" label
    ISSUE_NUMBER=$(gh issue list --state open --json number,labels --jq '[.[] | select(.labels | map(.name) | contains(["in-progress"]) | not)] | .[0].number' 2>/dev/null)
fi

if [ -z "$ISSUE_NUMBER" ] || [ "$ISSUE_NUMBER" = "null" ]; then
    echo "No available issues found with Todo status" >&2
    exit 1
fi

# Update issue: remove "todo" label (if present) and add "in-progress" label
gh issue edit "$ISSUE_NUMBER" --remove-label "todo" 2>/dev/null || true
gh issue edit "$ISSUE_NUMBER" --add-label "in-progress"

echo "$ISSUE_NUMBER"
