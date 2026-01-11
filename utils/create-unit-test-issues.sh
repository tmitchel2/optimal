#!/usr/bin/env bash
set -e

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
LIMIT=10

while [[ $# -gt 0 ]]; do
    case $1 in
        --limit)
            LIMIT="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

# Get least covered files
COVERAGE_DATA=$("$SCRIPT_DIR/get-least-covered-files.sh" --limit "$LIMIT")

# Create issues for each item
echo "$COVERAGE_DATA" | python3 << EOF
import json
import subprocess
import sys

data = json.loads('''$COVERAGE_DATA''')

created_issues = []

for item in data:
    name = item['name']
    title = f"Create unit tests for {name}"
    body = json.dumps(item, indent=2)

    # Create the issue using gh CLI
    try:
        result = subprocess.run(
            ['gh', 'issue', 'create', '--title', title, '--body', body],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            # Extract issue URL from output
            issue_url = result.stdout.strip()
            created_issues.append({
                "name": name,
                "url": issue_url,
                "status": "created"
            })
            print(f"Created issue: {title}", file=sys.stderr)
            print(f"  URL: {issue_url}", file=sys.stderr)
        else:
            created_issues.append({
                "name": name,
                "error": result.stderr.strip(),
                "status": "failed"
            })
            print(f"Failed to create issue for {name}: {result.stderr}", file=sys.stderr)
    except Exception as e:
        created_issues.append({
            "name": name,
            "error": str(e),
            "status": "failed"
        })
        print(f"Error creating issue for {name}: {e}", file=sys.stderr)

print(json.dumps(created_issues, indent=2))
EOF
