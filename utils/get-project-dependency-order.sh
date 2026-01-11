#!/usr/bin/env bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Function to check if a project is a test project
is_test_project() {
    local csproj_path="$1"
    if grep -q "Microsoft.NET.Test.Sdk\|MSTest\|xunit\|NUnit" "$csproj_path" 2>/dev/null; then
        echo "true"
    else
        echo "false"
    fi
}

# Function to get project references from a csproj file
get_project_refs() {
    local csproj_path="$1"
    grep -o 'Include="[^"]*\.csproj"' "$csproj_path" 2>/dev/null | sed 's/Include="//g' | sed 's/"//g' || true
}

# Get all projects in solution
PROJECTS=$(find "$PROJECT_ROOT/src" -name "*.csproj" -type f 2>/dev/null | sort)

# Create temporary files for tracking
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

# Initialize dependee counts
for proj in $PROJECTS; do
    proj_name=$(basename "$proj" .csproj)
    echo "0" > "$TEMP_DIR/${proj_name}.count"
    echo "$proj" > "$TEMP_DIR/${proj_name}.path"
done

# Count how many times each project is depended upon
for proj in $PROJECTS; do
    refs=$(get_project_refs "$proj")
    for ref in $refs; do
        ref_name=$(basename "$ref" .csproj)
        if [ -f "$TEMP_DIR/${ref_name}.count" ]; then
            current=$(cat "$TEMP_DIR/${ref_name}.count")
            echo $((current + 1)) > "$TEMP_DIR/${ref_name}.count"
        fi
    done
done

# Create sorted list by dependee count (ascending - fewer dependees first)
# This puts leaf nodes (projects with fewest dependees) first
SORTED_PROJECTS=""
for i in $(seq 0 10); do
    for proj in $PROJECTS; do
        proj_name=$(basename "$proj" .csproj)
        count=$(cat "$TEMP_DIR/${proj_name}.count" 2>/dev/null || echo "0")
        if [ "$count" -eq "$i" ]; then
            SORTED_PROJECTS="$SORTED_PROJECTS $proj_name"
        fi
    done
done

# Build JSON output
JSON="["
FIRST=true
for proj_name in $SORTED_PROJECTS; do
    proj_path=$(cat "$TEMP_DIR/${proj_name}.path")
    # Make path relative to project root
    rel_path="${proj_path#$PROJECT_ROOT/}"
    is_test=$(is_test_project "$proj_path")

    if [ "$FIRST" = true ]; then
        FIRST=false
    else
        JSON="$JSON,"
    fi
    JSON="$JSON{\"name\":\"$proj_name\",\"path\":\"$rel_path\",\"isTestProject\":$is_test}"
done
JSON="$JSON]"

echo "$JSON"
