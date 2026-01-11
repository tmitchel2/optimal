#!/usr/bin/env bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
PROJECT_PATH=""
LIMIT=5
DEBUG=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --limit)
            LIMIT="$2"
            shift 2
            ;;
        --debug)
            DEBUG=true
            shift
            ;;
        *)
            PROJECT_PATH="$1"
            shift
            ;;
    esac
done

log() {
    if [ "$DEBUG" = true ]; then
        echo "[DEBUG] $1" >&2
    fi
}

if [ -z "$PROJECT_PATH" ]; then
    echo "Usage: get-project-coverage.sh <project-path> [--limit N] [--debug]" >&2
    exit 1
fi

# Resolve project path
FULL_PROJECT_PATH="$PROJECT_ROOT/$PROJECT_PATH"
if [ ! -f "$FULL_PROJECT_PATH" ]; then
    echo "Project file not found: $FULL_PROJECT_PATH" >&2
    exit 1
fi

PROJECT_NAME=$(basename "$FULL_PROJECT_PATH" .csproj)
PROJECT_DIR=$(dirname "$FULL_PROJECT_PATH")

log "Project name: $PROJECT_NAME"
log "Project dir: $PROJECT_DIR"

# Create unique coverage folder
COVERAGE_ID="coverage-$(date +%s)-$$"
COVERAGE_DIR="$PROJECT_ROOT/.coverage/$COVERAGE_ID"
mkdir -p "$COVERAGE_DIR"

log "Coverage dir: $COVERAGE_DIR"

# Cleanup function
cleanup() {
    if [ "$DEBUG" = false ]; then
        rm -rf "$COVERAGE_DIR"
    else
        log "Keeping coverage dir for inspection: $COVERAGE_DIR"
    fi
}
trap cleanup EXIT

# Build the solution first
log "Building solution..."
dotnet build "$PROJECT_ROOT/src/Optimal.slnx" \
    --no-incremental \
    -p:UseSharedCompilation=false \
    -p:UseRazorBuildServer=false \
    /nodeReuse:false \
    --verbosity quiet >/dev/null 2>&1 || {
    echo "Build failed" >&2
    exit 1
}
log "Build complete"

# Run tests with coverage
log "Running tests with coverage..."
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
log "Tests complete"

# Find all coverage XML files
COVERAGE_FILES=$(find "$COVERAGE_DIR" -name "coverage.cobertura.xml" 2>/dev/null)

if [ -z "$COVERAGE_FILES" ]; then
    echo "No coverage files found" >&2
    exit 1
fi

log "Found coverage files:"
for f in $COVERAGE_FILES; do
    log "  $f"
done

# Create temporary file for aggregated class data
TEMP_FILE=$(mktemp)
trap "rm -f $TEMP_FILE; cleanup" EXIT

log "Processing coverage files..."

# Process coverage files and extract class data for the target project
for COVERAGE_FILE in $COVERAGE_FILES; do
    log "Processing: $COVERAGE_FILE"
    python3 << EOF >> "$TEMP_FILE"
import xml.etree.ElementTree as ET
import os
import json
import sys

coverage_file = "$COVERAGE_FILE"
project_name = "$PROJECT_NAME"
project_dir = "$PROJECT_DIR"
project_root = "$PROJECT_ROOT"
debug = "$DEBUG" == "true"

def log(msg):
    if debug:
        print(f"[PYTHON] {msg}", file=sys.stderr)

try:
    tree = ET.parse(coverage_file)
    root = tree.getroot()

    # Get sources to reconstruct full paths
    sources = [s.text.rstrip('/') for s in root.findall('.//source') if s.text]
    log(f"Sources: {sources}")

    packages = root.findall('.//package')
    log(f"Found {len(packages)} packages")

    for package in packages:
        pkg_name = package.get('name', '')
        log(f"Package: {pkg_name}")

        # Only process packages matching our project
        if pkg_name != project_name:
            log(f"  Skipping - doesn't match {project_name}")
            continue

        classes = package.findall('.//class')
        log(f"  Found {len(classes)} classes")

        for cls in classes:
            filename = cls.get('filename', '')
            name = cls.get('name', '')

            if not filename or not name:
                continue

            # Reconstruct full path from source + filename
            full_path = None
            for source in sources:
                test_path = os.path.join(source, filename)
                if os.path.exists(test_path):
                    full_path = test_path
                    break

            if not full_path:
                # Fallback: try with project directory
                full_path = os.path.join(project_dir, filename)

            # Make path relative to project root
            if full_path.startswith(project_root):
                rel_path = full_path[len(project_root)+1:]
            else:
                rel_path = full_path

            # Get line coverage data
            lines = cls.findall('.//line')
            covered = sum(1 for l in lines if int(l.get('hits', 0)) > 0)
            not_covered = sum(1 for l in lines if int(l.get('hits', 0)) == 0)
            total = covered + not_covered

            if total > 0:
                ratio = covered / total
                log(f"  Class: {name}, coverage: {ratio:.2%}")
                print(json.dumps({
                    "name": name,
                    "path": rel_path,
                    "covered": covered,
                    "notCovered": not_covered,
                    "ratio": ratio
                }))
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    import traceback
    traceback.print_exc(file=sys.stderr)
EOF
done

log "Aggregating results..."

# Check temp file contents
if [ "$DEBUG" = true ]; then
    log "Temp file line count: $(wc -l < "$TEMP_FILE")"
fi

# Aggregate duplicates (same class may appear in multiple coverage files)
# Take the best coverage for each class
python3 << EOF
import json
import sys

debug = "$DEBUG" == "true"

def log(msg):
    if debug:
        print(f"[AGGREGATE] {msg}", file=sys.stderr)

data = {}
with open("$TEMP_FILE", 'r') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            item = json.loads(line)
            key = item['name'] + '|' + item['path']
            if key not in data or item['ratio'] > data[key]['ratio']:
                data[key] = item
        except Exception as e:
            log(f"Error parsing line: {e}")

log(f"Found {len(data)} unique classes")

# Convert to list and sort by coverage ratio (ascending - lowest coverage first)
items = list(data.values())
items.sort(key=lambda x: x['ratio'])

log(f"After sorting: {len(items)} items")

# Filter out items with coverage ratio >= 0.9
items = [i for i in items if i['ratio'] < 0.9]

log(f"After filtering >=90%: {len(items)} items")

# Apply limit
limit = $LIMIT
items = items[:limit]

log(f"After limit: {len(items)} items")

# Format output
result = []
for item in items:
    result.append({
        "name": item['name'],
        "path": item['path'],
        "lines": {
            "coverageRatio": round(item['ratio'], 4),
            "covered": item['covered'],
            "notCovered": item['notCovered']
        }
    })

print(json.dumps(result))
EOF
