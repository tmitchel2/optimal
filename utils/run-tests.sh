#!/bin/bash
set -e

# Disable MSBuild node reuse to prevent hanging processes
export MSBUILDDISABLENODEREUSE=1

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Run all tests
dotnet test "$PROJECT_ROOT/src/Optimal.slnx" \
    -p:UseSharedCompilation=false \
    -p:UseRazorBuildServer=false \
    /nodeReuse:false \
    --verbosity quiet
