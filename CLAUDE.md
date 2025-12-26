# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Optimal** is a .NET 10.0 library project written in C#. The project uses strict code quality standards with Roslynator analyzers and treats warnings as errors.

## Build & Test Commands

### Prerequisites
```bash
# Ensure MSBuild nodes are cleaned up before running commands
export MSBUILDDISABLENODEREUSE=1

# Restore dotnet tools (required for code coverage reports)
dotnet tool restore
```

### Building
```bash
# Build the solution (always run from /src directory)
cd src
dotnet build Optimal.slnx -p:UseSharedCompilation=false -p:UseRazorBuildServer=false --nodeReuse:false --verbosity quiet
```

### Testing & Coverage
```bash
# Run tests with code coverage (from /src directory)
cd src
./build.sh

# Or manually:
dotnet test --collect:"XPlat Code Coverage" \
  --settings:"settings.runsettings" \
  --filter:"TestCategory!=Integration" \
  --results-directory:"./bin/test/results" \
  Optimal.slnx

# Generate coverage report
dotnet reportgenerator \
  -reports:"./bin/test/results/**/coverage.cobertura.xml" \
  -targetdir:"./bin/test/results/codecoverage/"
```

The build script outputs a file URL to view the coverage report in a browser.

### Cleaning
```bash
cd src
dotnet clean Optimal.slnx --nodeReuse:false
```

## Architecture & Code Standards

### Project Structure
- **Solution**: `src/Optimal.slnx` (XML-based solution format)
- **Main Project**: `src/Optimal/Optimal.csproj` - .NET 10.0 library with nullable reference types enabled
- **Build Configuration**: `src/Directory.Build.props` - Centralized build properties with Nerdbank.GitVersioning and Roslynator analyzers

### Folder Organization
- Folders under projects should be **flat and only one level deep**
- For namespaces with multiple segments, use dots (`.`) in folder names rather than nested directories
  - Example: `Optimal.Data.Models` → folder named `Data.Models/` not `Data/Models/`
- Each class, record, enum, etc. should be in its own file

### Naming Conventions
Per `.editorconfig`:
- **Instance fields**: camelCase with `_` prefix (e.g., `_myField`)
- **Static fields**: camelCase with `s_` prefix (e.g., `s_cache`)
- **Constants**: PascalCase
- **Methods/Properties**: PascalCase (including test methods - use `CanGetDateTimePropertyAsString`, NOT `Can_Get_DateTime_Property_As_String`)
- **Parameters/Locals**: camelCase
- **Non-private readonly/static fields**: PascalCase

### Code Quality
- **Warnings as errors**: Enabled via `TreatWarningsAsErrors=true`
- **Nullable**: Enabled - all reference types must be explicitly nullable or non-nullable
- **Var preference**: Use `var` everywhere (enforced as warning)
- **Braces**: Required for multi-line blocks
- **Documentation**: XML documentation files auto-generated but doc comments (CS1591) warnings suppressed
- **Formatting**: 4 spaces for C#, UTF-8 BOM encoding, final newline required

### MSBuild Flags
When running dotnet commands, always include these flags to ensure processes terminate properly:
```bash
-p:UseSharedCompilation=false -p:UseRazorBuildServer=false --nodeReuse:false
```

### Test Coverage Configuration
- Tests run with `settings.runsettings` which excludes integration tests by default
- Coverage excludes: `*Tests` assemblies, `*.g.cs` generated files, compiler-generated display classes
- Coverage includes: `[Optimal]*` and `[Optimal*]*` assemblies
- Run tests on single CPU (`MaxCpuCount=1`) to avoid conflicts

## Testing Conventions

- **Framework**: Appears to use XPlat Code Coverage with standard .NET testing
- **Test filtering**: Use `TestCategory!=Integration` to exclude integration tests
- **Helper methods**: Should be static when possible
- **Mocking**: Manual mock classes are preferred over mocking frameworks like Moq

## Working with dotnet Commands

### Important Notes
- **Working directory**: dotnet commands require the current working directory to contain a project/solution file, OR pass the filepath as a positional parameter
- **DO NOT redirect output** to /dev/null (`> /dev/null 2>&1`) - let output display
- **Check exit codes**: Use `BUILD_EXIT=$?` to check command success
- **MSBUILDDISABLENODEREUSE**: Always export this environment variable before running dotnet commands

### Example Command Pattern
```bash
export MSBUILDDISABLENODEREUSE=1
cd src
dotnet build Optimal.slnx --nodeReuse:false
BUILD_EXIT=$?
if [ $BUILD_EXIT -ne 0 ]; then
  exit $BUILD_EXIT
fi
```

## IDE & Tooling

- **.NET SDK**: 10.0.100 (with `latestFeature` rollForward)
- **Tools**: dotnet-trace (8.0.452401), reportgenerator (5.2.0)
- **VS Code**: Configured with .NET Core Attach debugger
- **Dev Container**: Available with Claude Code, ESLint, Prettier, GitLens extensions

## Claude Code Integration

This repository has extensive Claude Code automation:

### Custom Skills
- `dotnet-coder`: Specialized skill for C# and .NET development
- `github-user`: Interact with GitHub repositories, issues, PRs
- `github-issue-implementer`: Implement GitHub issues
- `test-implementer`: Create unit and integration tests
- `worktree-user`: Work with git worktrees for parallel development

### Custom Commands
- `/create-issue [area-of-code]`: Create a GitHub issue
- `/create-unit-test-issue [area-of-code]`: Create a unit testing issue
- `/implement-issue [issue-number]`: Implement a specific GitHub issue

### Shortcuts
- `claude-yolo.sh`: Run Claude Code without permission prompts
- `copilot-yolo.sh`: Run GitHub Copilot CLI with all tools enabled
