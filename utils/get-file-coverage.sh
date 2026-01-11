#!/usr/bin/env bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
TARGET_FILE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        *)
            TARGET_FILE="$1"
            shift
            ;;
    esac
done

if [ -z "$TARGET_FILE" ]; then
    echo "Usage: get-file-coverage.sh <file-path>" >&2
    exit 1
fi

# Resolve target file path
FULL_TARGET_PATH="$PROJECT_ROOT/$TARGET_FILE"
if [ ! -f "$FULL_TARGET_PATH" ]; then
    echo "Target file not found: $FULL_TARGET_PATH" >&2
    exit 1
fi

TARGET_FILENAME=$(basename "$TARGET_FILE" .cs)

# Find associated test file
# Pattern: src/Optimal/Control.Solvers/HermiteSimpsonSolver.cs
#       -> src/Optimal.Tests/Control.Solvers.Tests/HermiteSimpsonSolverTests.cs
find_test_file() {
    local target="$1"
    local target_dir=$(dirname "$target")
    local target_name=$(basename "$target" .cs)

    # Convert path: src/Optimal/Control.Solvers -> src/Optimal.Tests/Control.Solvers.Tests
    local test_dir=$(echo "$target_dir" | sed 's|src/Optimal/|src/Optimal.Tests/|' | sed 's|$|.Tests|')
    local test_file="$test_dir/${target_name}Tests.cs"

    if [ -f "$PROJECT_ROOT/$test_file" ]; then
        echo "$test_file"
        return 0
    fi

    # Try alternative patterns
    # Pattern: NonLinear/ConvergenceMonitor.cs -> NonLinear.Tests/ConvergenceMonitorTests.cs
    test_dir=$(echo "$target_dir" | sed 's|src/Optimal/||')
    test_file="src/Optimal.Tests/${test_dir}.Tests/${target_name}Tests.cs"

    if [ -f "$PROJECT_ROOT/$test_file" ]; then
        echo "$test_file"
        return 0
    fi

    return 1
}

TEST_FILE=$(find_test_file "$TARGET_FILE" || echo "")

if [ -z "$TEST_FILE" ]; then
    python3 << EOF
import json
result = {
    "file": "$TARGET_FILE",
    "error": "No test file found for this target file",
    "summary": None,
    "methodsAndProperties": []
}
print(json.dumps(result, indent=2))
EOF
    exit 0
fi

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

# Run tests with coverage - filter to just the test file's tests
TEST_CLASS=$(basename "$TEST_FILE" .cs)
dotnet test "$PROJECT_ROOT/src/Optimal.slnx" \
    -p:UseSharedCompilation=false \
    -p:UseRazorBuildServer=false \
    /nodeReuse:false \
    --collect:"XPlat Code Coverage" \
    --results-directory "$COVERAGE_DIR" \
    --settings "$PROJECT_ROOT/src/settings.runsettings" \
    --filter "FullyQualifiedName~$TEST_CLASS" \
    --verbosity quiet >/dev/null 2>&1 || {
    echo "Tests failed" >&2
    exit 1
}

# Find all coverage XML files
COVERAGE_FILES=$(find "$COVERAGE_DIR" -name "coverage.cobertura.xml" 2>/dev/null)

if [ -z "$COVERAGE_FILES" ]; then
    python3 << EOF
import json
result = {
    "file": "$TARGET_FILE",
    "error": "No coverage data generated",
    "summary": None,
    "methodsAndProperties": []
}
print(json.dumps(result, indent=2))
EOF
    exit 0
fi

# Process coverage files
python3 << EOF
import xml.etree.ElementTree as ET
import os
import json
import glob

coverage_dir = "$COVERAGE_DIR"
target_file = "$TARGET_FILE"
target_filename = "$TARGET_FILENAME"
project_root = "$PROJECT_ROOT"

# Find all coverage files
coverage_files = glob.glob(os.path.join(coverage_dir, "**/coverage.cobertura.xml"), recursive=True)

# Aggregate data from all coverage files
best_class_data = {}

for coverage_file in coverage_files:
    try:
        tree = ET.parse(coverage_file)
        root = tree.getroot()

        # Get sources
        sources = [s.text.rstrip('/') for s in root.findall('.//source') if s.text]

        for cls in root.findall('.//class'):
            filename = cls.get('filename', '')
            class_name = cls.get('name', '')

            if not filename:
                continue

            # Build full path
            full_path = None
            for source in sources:
                test_path = os.path.join(source, filename)
                if os.path.exists(test_path):
                    full_path = test_path
                    break

            if not full_path:
                continue

            # Make relative to project root
            if full_path.startswith(project_root):
                rel_path = full_path[len(project_root)+1:]
            else:
                rel_path = filename

            # Check if this is our target file
            if rel_path != target_file:
                continue

            # Process methods
            methods_data = {}
            for method in cls.findall('.//method'):
                method_name = method.get('name', '')
                if not method_name:
                    continue

                lines = []
                for line in method.findall('.//line'):
                    line_num = int(line.get('number', 0))
                    hits = int(line.get('hits', 0))
                    branch = line.get('branch', 'False') == 'True'

                    if hits > 0:
                        state = "COVERED"
                    else:
                        state = "UNCOVERED"

                    lines.append({
                        "lineNumber": line_num,
                        "state": state,
                        "hits": hits
                    })

                covered = sum(1 for l in lines if l['state'] == 'COVERED')
                uncovered = sum(1 for l in lines if l['state'] == 'UNCOVERED')
                total = covered + uncovered

                # Get complexity and crap score from method attributes
                complexity = int(method.get('complexity', 1))
                line_rate = float(method.get('line-rate', 0))

                # Calculate CRAP score: C^2 * (1 - coverage)^3 + C
                coverage_ratio = line_rate
                crap_score = complexity ** 2 * (1 - coverage_ratio) ** 3 + complexity

                method_key = method_name
                if method_key not in methods_data or covered > methods_data[method_key]['covered']:
                    methods_data[method_key] = {
                        "name": method_name,
                        "lineCoverage": {
                            "covered": covered,
                            "uncovered": uncovered,
                            "coverable": total,
                            "coverableAndUncoverable": total,
                            "coveragePercentage": round(coverage_ratio * 100, 2),
                            "cyclomaticComplexity": complexity,
                            "crapScore": round(crap_score, 2)
                        },
                        "lines": sorted(lines, key=lambda x: x['lineNumber']),
                        "covered": covered
                    }

            class_key = class_name
            if class_key not in best_class_data:
                best_class_data[class_key] = methods_data
            else:
                # Merge, taking best coverage
                for method_name, method_data in methods_data.items():
                    if method_name not in best_class_data[class_key] or \
                       method_data['covered'] > best_class_data[class_key][method_name]['covered']:
                        best_class_data[class_key][method_name] = method_data

    except Exception as e:
        pass

# Aggregate all methods
all_methods = []
for class_name, methods_data in best_class_data.items():
    for method_name, method_data in methods_data.items():
        # Remove internal 'covered' field
        method_copy = {k: v for k, v in method_data.items() if k != 'covered'}
        # Clean up lines - remove hits field
        method_copy['lines'] = [{"lineNumber": l['lineNumber'], "state": l['state']} for l in method_copy['lines']]
        all_methods.append(method_copy)

# Calculate summary
total_covered = sum(m['lineCoverage']['covered'] for m in all_methods)
total_uncovered = sum(m['lineCoverage']['uncovered'] for m in all_methods)
total_lines = total_covered + total_uncovered

if total_lines > 0:
    coverage_pct = round(total_covered / total_lines * 100, 2)
else:
    coverage_pct = 0

# Calculate average complexity and crap
if all_methods:
    avg_complexity = round(sum(m['lineCoverage']['cyclomaticComplexity'] for m in all_methods) / len(all_methods), 2)
    avg_crap = round(sum(m['lineCoverage']['crapScore'] for m in all_methods) / len(all_methods), 2)
else:
    avg_complexity = 0
    avg_crap = 0

result = {
    "file": target_file,
    "summary": {
        "lineCoverage": {
            "covered": total_covered,
            "uncovered": total_uncovered,
            "coverable": total_lines,
            "coverableAndUncoverable": total_lines,
            "coveragePercentage": coverage_pct,
            "cyclomaticComplexity": avg_complexity,
            "crapScore": avg_crap
        }
    },
    "methodsAndProperties": all_methods
}

print(json.dumps(result, indent=2))
EOF
