# Initialise utils

## Important

- Create each script in order, verify that each one works correctly before moving onto the next, get confirmation to move onto the next script.
- If the script already exists then just verify that it works correctly before moving onto the next item, if it does not work correctly then delete the file and create the script from scratch.
- When writing scripts that use dotnet make sure to disable MSBuild node reuse to prevent hanging processes (e.g. export MSBUILDDISABLENODEREUSE=1)
- Dotnet build requires several parameters to sport lingering processes, e.g. dotnet build src/Optimal.slnx --no-incremental -p:UseSharedCompilation=false -p:UseRazorBuildServer=false /nodeReuse:false --verbosity quiet
- Dotnet test should not use the --no-build flag, it should rebuild if required, it should use similar flags to build, e.g. dotnet test src/Optimal.slnx --no-incremental -p:UseSharedCompilation=false -p:UseRazorBuildServer=false /nodeReuse:false --verbosity quiet

## Your task

Create the following utility scripts in order...

### run-build.sh

Create a utility script (called run-build.sh) in ./utils which can run build all the projects.

### run-tests.sh

Create a utility script (called run-tests.sh) in ./utils which can run all unit tests.

### get-solution-coverage.sh

Create a utility script (called get-solution-coverage.sh) in ./utils which runs code coverage using all unit tests. The script should store any intermediate coverage files to a uniquely named folder under ".coverage" folder which is to be located at the project root. The script should output JSON in the format { lineCoveragePercentage: 0, branchCoveragePercentage: 0 } replacing the values lineCoveragePercentage and branchCoveragePercentage with values extracted from the coverage report. Verify the output to ensure both values are non zero. Ensure that all coverage files are used to calculate the total coverage, there could be more than just one file. The script should cleanup any temporary coverage files after the coverage has been calculated. The script should explicitly build the whole library / app before running code coverage on the project. Code coverage should use ./src/settings.runsettings. The script should fail and bubble up any error message if any of the internally called scripts fail. If the whole script executes correctly then none of the internal scripts should output any log messages, only the final JSON result should be returned along with a valid exit code.

### get-project-dependency-order.sh

Create a utility script (called get-project-dependency-order.sh)in ./utils which returns a JSON array, where each item in the array has the format {name: "", path: "", isTestProject: true}. The output should be ordered to show the projects with the fewest dependees at the start, and the projects with the most dependees at the end. To determine this order the script should internally determine the project graph hierarchy and then select the leaf nodes first and work its way up the tree to the root node. The script should fail and bubble up any error message if any of the internally called scripts fail. If the whole script executes correctly then none of the internal scripts should output any log messages, only the final JSON result should be returned along with a valid exit code.

### get-project-coverage.sh

Create a utility script (called get-project-coverage.sh) in ./utils which runs code coverage for all the unit tests within a single project, it should return a simple JSON parsable array. The array item should be in the format { name: "MyName", path: "", lines: { coverageRatio: 0.5, covered: 123, notCovered: 123 } }. The "name" should be the name of the class, the "path" should be the full file path relative to root. The array should be ordered so that items with smaller coverageRatio appear at the start. The array should be filtered so that items with coverageRatio above 0.9 are removed. The script should accept an optional limit parameter which reduces the final array output length to that size, the default limit value should be 5. The script should accept the full project filepath relative to the root. Ensure that the output "path" from the get-project-dependency-order.sh script works as the input to this script. The script should store any intermediate coverage files to a uniquely named folder under ".coverage" folder which is to be located at the project root. When testing the script make sure the project passed in as a parameter is not a test project. Ensure that all coverage files are used to calculate the total coverage, there could be more than just one file. The script should cleanup any temporary coverage files after the coverage has been calculated. The script should explicitly build the whole library / app before running code coverage on the project. Code coverage should use ./src/settings.runsettings. The script should fail and bubble up any error message if any of the internally called scripts fail. If the whole script executes correctly then none of the internal scripts should output any log messages, only the final JSON result should be returned along with a valid exit code.

### get-least-covered-files.sh

Create a utility script (called get-least-covered-files.sh) in ./utils which accepts an optional limit parameter. It should internally run the get-project-dependency-order.sh script, then it should filter out any test projects (where isTestProject is true). It should then run the get-project-coverage.sh script on each remaining item in the list
. It should combine the responses from get-project-coverage.sh, sort by "nonCovered" descending and filter out items as follows, items with "notCovered" values of 0, items which are generated by the compiler (analyse patterns of compiler generated classes to determine this). Then it should append the remaining items which have non zero "nonCovered" into an array. If array length exceeds the passed in limit value or all projects have been searched then the script should return the array with its length truncated to the limit value. The output format should be the same as the get-project-coverage.sh script. Verify that the script generates the correct output. Only use the utils scripts I've mentioned here for this task. Ensure the script file inlines any new script rather than create any new external scripts.

### get-file-coverage.sh

Create a utility script (called get-file-coverage.sh) in ./utils which takes the filepath of a file as input. That argument is the target file to be covered. There may or may not be an associated test file dedicated to providing unit testing and coverage to that target file. If a unit test file is found then code coverage should be run just for that target file and the response returned. If a unit test file is not found then the output should state that the target file has no test coverage. The file path passed to this script should be relative to the root folder of the project. The script should store any intermediate coverage files to a uniquely named folder under ".coverage" folder which is to be located at the project root. Please note the following issues which can arise when implementing this script:

- Coverage XML may contain just the filename, relative path, or full path so ensure that matching works for those cases.
- Aggregate all classes within the same file for each coverage run.
- Filter files to only include those from the project being tested
- Take the maximum coverage when multiple duplicate coverage files exist from the same test run

The output should be in formatted and indented JSON in the format:

```
{
  file: "<FILE_PATH>",
  summary: {
    lineCoverage: {
      covered: 0,
      uncovered: 0,
      coverable: 0,
      coverableAndUncoverable: 0,
      coveragePercentage: 0,
      cyclomaticComplexity: 0,
      crapScore: 0
    },
  }
  methodsAndProperties: [
    {
      name: "<METHOD_PROP_OR_FIELD_NAME>",
      lineCoverage: {
        covered: 0,
        uncovered: 0,
        coverable: 0,
        coverableAndUncoverable: 0,
        coveragePercentage: 0,
        cyclomaticComplexity: 0,
        crapScore: 0
      },
      lines: [
        {
          lineNumber: 0,
          state: "COVERED" | "UNCOVERED" | "UNCOVERABLE"
        }
      ]
    }
  ]
}
```

### create-unit-test-issues.sh

Create a utility script (called create-unit-test-issues.sh) in ./utils which runs get-least-covered-files.sh passing in a limit parameter of 10. For each item in the returned array you should create a new github issue with the title "Create unit tests for {NAME}" where {NAME} is replaced with the value of the "name" property on the item. The description of the issue should be the JSON string for this item. Ensure the script works correctly by running with limit 1 verifying that the issue has been created, once verified the temporary issue should then be set to done.

### begin-issue.sh

Create a utility script (called begin-issue.sh) in ./utils which finds a GitHub issue in Optimal project which has a status of "Todo". It should update the issue status to "In Progress" and return the issue number in the script output. To verify that the script works you should run it against an existing issue, if it works then set the test issue status back to "Todo" again.

### get-crap-scores.sh

Create a utility script (called get-crap-scores.sh) in ./utils which calculates the CRAP scores for all types and functions, orders them from worst to best and returns by default the top 10 worst functions. The script should compile the whole project and run all tests with coverage, it should store any intermediate coverage files to a uniquely named folder under ".coverage" folder which is to be located at the project root. Then the script should calculate CRAP scores for each function across all types, order the results and then truncate the list to the first 10 items. The script should output a JSON list of objects in the format { typeName: "", methodName: "", filePath: "", crapScore: 0 }, replacing the values with values extracted from the coverage report and the calculated CRAP score. Verify the output to ensure the results look appropriate. Ensure that all coverage files are used to calculate the total coverage, there could be more than just one file. The script should cleanup any temporary coverage files after the coverage has been calculated. The script should explicitly build the whole library / app before running code coverage on the project. Code coverage should use ./src/settings.runsettings. The script should fail and bubble up any error message if any of the internally called scripts fail. If the whole script executes correctly then none of the internal scripts should output any log messages, only the final JSON result should be returned along with a valid exit code. The script should take an optional flag --json which ensures that only the JSON result is output to the console, if this flag is omitted then the script should output a message at the start to state that it is running and one at the end before the result output to state that it has finished.

### help.sh

Create a utility script (called help.sh) in ./utils which outputs helpful information about the scripts in ./utils. It should describe how to invoke each script, describe usage of any arguments. It should be extremely consise, each script should be described on just a single line.
