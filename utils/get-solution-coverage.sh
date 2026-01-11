#!/bin/bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Create unique coverage folder
COVERAGE_ID="coverage-$(date +%s)-$$"
COVERAGE_DIR="$PROJECT_ROOT/.coverage/$COVERAGE_ID"
mkdir -p "$COVERAGE_DIR"

# Cleanup function
cleanup() {
    rm -rf "$COVERAGE_DIR"
}
trap cleanup EXIT

# Build the solution first
dotnet build "$PROJECT_ROOT/src/Optimal.slnx" \
    --no-incremental \
    -p:UseSharedCompilation=false \
    -p:UseRazorBuildServer=false \
    /nodeReuse:false \
    --verbosity quiet >/dev/null 2>&1 || {
    echo "Build failed" >&2
    exit 1
}

# Run tests with coverage
dotnet test "$PROJECT_ROOT/src/Optimal.slnx" \
    -p:UseSharedCompilation=false \
    -p:UseRazorBuildServer=false \
    /nodeReuse:false \
    --collect:"XPlat Code Coverage" \
    --results-directory "$COVERAGE_DIR" \
    --settings "$PROJECT_ROOT/src/settings.runsettings" \
    --verbosity quiet >/dev/null 2>&1 || {
    echo "Tests failed" >&2
    exit 1
}

# Find all coverage XML files
COVERAGE_FILES=$(find "$COVERAGE_DIR" -name "coverage.cobertura.xml" 2>/dev/null)

if [ -z "$COVERAGE_FILES" ]; then
    echo "No coverage files found" >&2
    exit 1
fi

# Calculate totals from all coverage files
TOTAL_LINES_COVERED=0
TOTAL_LINES_VALID=0
TOTAL_BRANCHES_COVERED=0
TOTAL_BRANCHES_VALID=0

for COVERAGE_FILE in $COVERAGE_FILES; do
    # Extract line coverage
    LINES_COVERED=$(grep -o 'lines-covered="[0-9]*"' "$COVERAGE_FILE" | head -1 | grep -o '[0-9]*')
    LINES_VALID=$(grep -o 'lines-valid="[0-9]*"' "$COVERAGE_FILE" | head -1 | grep -o '[0-9]*')

    # Extract branch coverage
    BRANCHES_COVERED=$(grep -o 'branches-covered="[0-9]*"' "$COVERAGE_FILE" | head -1 | grep -o '[0-9]*')
    BRANCHES_VALID=$(grep -o 'branches-valid="[0-9]*"' "$COVERAGE_FILE" | head -1 | grep -o '[0-9]*')

    TOTAL_LINES_COVERED=$((TOTAL_LINES_COVERED + ${LINES_COVERED:-0}))
    TOTAL_LINES_VALID=$((TOTAL_LINES_VALID + ${LINES_VALID:-0}))
    TOTAL_BRANCHES_COVERED=$((TOTAL_BRANCHES_COVERED + ${BRANCHES_COVERED:-0}))
    TOTAL_BRANCHES_VALID=$((TOTAL_BRANCHES_VALID + ${BRANCHES_VALID:-0}))
done

# Calculate percentages
if [ "$TOTAL_LINES_VALID" -gt 0 ]; then
    LINE_COVERAGE=$(echo "scale=2; $TOTAL_LINES_COVERED * 100 / $TOTAL_LINES_VALID" | bc)
else
    LINE_COVERAGE=0
fi

if [ "$TOTAL_BRANCHES_VALID" -gt 0 ]; then
    BRANCH_COVERAGE=$(echo "scale=2; $TOTAL_BRANCHES_COVERED * 100 / $TOTAL_BRANCHES_VALID" | bc)
else
    BRANCH_COVERAGE=0
fi

# Output JSON
echo "{ \"lineCoveragePercentage\": $LINE_COVERAGE, \"branchCoveragePercentage\": $BRANCH_COVERAGE }"
