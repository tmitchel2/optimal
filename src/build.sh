cd "$(dirname "$0")"

rm -r ./bin/test/results
dotnet test --collect:"XPlat Code Coverage" --settings:"settings.runsettings" --filter:"TestCategory!=Integration" --results-directory:"./bin/test/results" ./Optimal.slnx

dotnet tool restore
dotnet reportgenerator -reports:"./bin/test/results/**/coverage.cobertura.xml" -targetdir:"./bin/test/results/codecoverage/"

echo "file://$(pwd)/bin/test/results/codecoverage/index.html"