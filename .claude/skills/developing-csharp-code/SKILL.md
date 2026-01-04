---
name: developing-csharp-code
description: You are an expert developer using the CSharp (C#) language and using dotnet tooling.
---

# Developing CSharp Code Skill

This skill provides additional information on how to work with CSharp (C#) and Dotnet.

## CSharp Language Best Practises

- Strive to create "Pure" functions, they should be deterministic, have no side effects, referencially transparent.
- Strive to use immutable data.
- Prefer smaller functions, the should have focused responsibility, doing one thing well.
- Each class, record, enum, etc should be within its own file.
- Use `var` everywhere, enforced as warning.
- Formatting, use 4 spaces for C#, UTF-8 BOM encoding, final newline required.
- Treat warnings as errors, enabled via `TreatWarningsAsErrors=true`
- Enable `Nullable`, make sure all reference types are explicitly nullable or non-nullable.
- Folders under projects should be **flat and only one level deep**
- For namespaces with multiple segments, use dots (`.`) in folder names rather than nested directories. e.g. `Optimal.Data.Models` folder is named `Data.Models/` not `Data/Models/`.
- Use Linq instead of loops.
- Reduce nesting / branch depth to an absolute minimum.  Loops within loops should be avoided, especially within the same function.  Branching within branching should be avoided, especially within the same function.
- Helper methods and classes should not implicitly default instances to particular values, and should not implicity fallback to alternate values if errors / exceptions occur.
- Failing fast is preferred rather than catching exceptions and attempting to fallback to alternate approaches.

## CSharp Workflow Best Practises

- Use a TDD approach for working.

## CSharp Coder Notes

- dotnet build needs the following flags to ensure the process stops and returns immediately on completion.  -p:UseSharedCompilation=false -p:UseRazorBuildServer=false nodeReuse:false --verbosity quiet
- Folders under projects should be flat and only one level deep, if the namespace of a type / file has multiple segments then the folder should use dots "." to express hierarchy rather than nested filesystem folders.
- Ensure that "export MSBUILDDISABLENODEREUSE=1" is run before any using and dotnet commands to ensure the called process finishes.
- dotnet commands require the current working directory to contain a project or solution file, or the filepath to one must be passed in as a positional parameter. E.g. dotnet clean src/GraphlessDB.sln --nodereuse:false or dotnet build src/GraphlessDB.sln --nodereuse:false
- Do not redirect output of dotnet commands to null using "> /dev/null 2>&1"
- Use BUILD_EXIT=$? to check the output of dotnet commands and exit the script with an error if one had occurred
- Follow existing project naming conventions, use PascalCase without underscores for method names including test methods (e.g., CanGetDateTimePropertyAsString and not Can_Get_DateTime_Property_As_String).
- Helper methods in test classes should be static when possible to follow project conventions.
- Manual mock classes are preferred over Moq framework in this project.
