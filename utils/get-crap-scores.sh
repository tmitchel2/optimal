#!/usr/bin/env bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
LIMIT=10
JSON_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --limit)
            LIMIT="$2"
            shift 2
            ;;
        --json)
            JSON_ONLY=true
            shift
            ;;
        *)
            shift
            ;;
    esac
done

log() {
    if [ "$JSON_ONLY" = false ]; then
        echo "$1" >&2
    fi
}

log "Running CRAP score analysis..."

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

# Find all coverage XML files
COVERAGE_FILES=$(find "$COVERAGE_DIR" -name "coverage.cobertura.xml" 2>/dev/null)

if [ -z "$COVERAGE_FILES" ]; then
    echo "No coverage files found" >&2
    exit 1
fi

log "Calculating CRAP scores..."

# Process coverage files and calculate CRAP scores
python3 << EOF
import xml.etree.ElementTree as ET
import os
import json
import glob

coverage_dir = "$COVERAGE_DIR"
project_root = "$PROJECT_ROOT"
limit = $LIMIT

# Find all coverage files
coverage_files = glob.glob(os.path.join(coverage_dir, "**/coverage.cobertura.xml"), recursive=True)

# Aggregate method data
method_data = {}

for coverage_file in coverage_files:
    try:
        tree = ET.parse(coverage_file)
        root = tree.getroot()

        # Get sources
        sources = [s.text.rstrip('/') for s in root.findall('.//source') if s.text]

        for cls in root.findall('.//class'):
            class_name = cls.get('name', '')
            filename = cls.get('filename', '')

            if not class_name or not filename:
                continue

            # Build full path
            full_path = None
            for source in sources:
                test_path = os.path.join(source, filename)
                if os.path.exists(test_path):
                    full_path = test_path
                    break

            if not full_path:
                full_path = filename

            # Make relative to project root
            if full_path.startswith(project_root):
                rel_path = full_path[len(project_root)+1:]
            else:
                rel_path = filename

            for method in cls.findall('.//method'):
                method_name = method.get('name', '')
                if not method_name:
                    continue

                # Get complexity and coverage
                complexity = int(method.get('complexity', 1))
                line_rate = float(method.get('line-rate', 0))

                # Calculate CRAP score: C^2 * (1 - coverage)^3 + C
                coverage = line_rate
                crap_score = complexity ** 2 * (1 - coverage) ** 3 + complexity

                # Skip trivial methods
                if complexity <= 1 and coverage >= 1.0:
                    continue

                key = f"{class_name}|{method_name}|{rel_path}"

                # Take the best coverage (lowest CRAP) for duplicates
                if key not in method_data or crap_score < method_data[key]['crapScore']:
                    method_data[key] = {
                        "typeName": class_name,
                        "methodName": method_name,
                        "filePath": rel_path,
                        "crapScore": round(crap_score, 2),
                        "complexity": complexity,
                        "coverage": round(coverage * 100, 2)
                    }

    except Exception as e:
        pass

# Sort by CRAP score (worst first - highest CRAP is worst)
items = list(method_data.values())
items.sort(key=lambda x: x['crapScore'], reverse=True)

# Apply limit
items = items[:limit]

# Format output (remove internal fields)
result = []
for item in items:
    result.append({
        "typeName": item['typeName'],
        "methodName": item['methodName'],
        "filePath": item['filePath'],
        "crapScore": item['crapScore']
    })

print(json.dumps(result))
EOF

log "CRAP score analysis complete."
