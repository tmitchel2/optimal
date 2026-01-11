#!/usr/bin/env bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

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

# Get project dependency order
PROJECTS=$("$SCRIPT_DIR/get-project-dependency-order.sh")

# Create temporary file for all coverage data
TEMP_FILE=$(mktemp)
trap "rm -f $TEMP_FILE" EXIT

# Filter out test projects and run coverage for each
echo "$PROJECTS" | python3 << EOF
import json
import subprocess
import sys
import os

projects = json.loads('''$PROJECTS''')
script_dir = "$SCRIPT_DIR"
limit = $LIMIT

all_items = []

for project in projects:
    if project.get('isTestProject', False):
        continue

    project_path = project['path']

    # Run get-project-coverage.sh for this project
    # Use a high limit to get all items, we'll filter later
    try:
        result = subprocess.run(
            [os.path.join(script_dir, 'get-project-coverage.sh'), project_path, '--limit', '1000'],
            capture_output=True,
            text=True,
            timeout=600
        )
        if result.returncode == 0 and result.stdout.strip():
            items = json.loads(result.stdout.strip())
            all_items.extend(items)
    except Exception as e:
        print(f"Error processing {project_path}: {e}", file=sys.stderr)

# Filter out compiler-generated classes (patterns like <>c__DisplayClass, <Module>, etc.)
def is_compiler_generated(name):
    patterns = [
        '<>c__DisplayClass',
        '<>c',
        '<Module>',
        '__StaticArrayInit',
        '<PrivateImplementationDetails>',
        'EmbeddedAttribute',
        'NullableAttribute',
        'NullableContextAttribute',
        'RefSafetyRulesAttribute',
        '/<>',  # Lambda display classes
    ]
    for pattern in patterns:
        if pattern in name:
            return True
    return False

# Filter items
filtered_items = []
for item in all_items:
    # Skip items with notCovered = 0
    if item['lines']['notCovered'] == 0:
        continue

    # Skip compiler-generated classes
    if is_compiler_generated(item['name']):
        continue

    filtered_items.append(item)

# Sort by notCovered descending
filtered_items.sort(key=lambda x: x['lines']['notCovered'], reverse=True)

# Apply limit
filtered_items = filtered_items[:limit]

print(json.dumps(filtered_items))
EOF
