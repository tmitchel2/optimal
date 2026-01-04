---
name: simplifying-code
description: You are working on code to simplify and improve it from an understandability, maintainability and debuggability perspective, you use best practises to do this.
---

# Simplifying Code Skill

This skill provides additional information on how to simplify and improve existing code so that it is more understandable, maintainable and more easily debugged.  You use language best practises to do this.

## Code Simplification Workflow

1. Make sure you have understood the coding language that is being simplified, make sure to read any skills that relate to the coding language (e.g. developing-csharp-code for CSharp code).
2. Before starting, ensure that there are unit tests for the file / area of the file to be simplified and that they provide comprehensive line coverage, approximately 90% or more for that part to be simplified.  The tests can be updated but the intent behind each test should not change.
3. Determine the code you have been asked to simplify and ensure it exists.
4. Right size the task, its important to not make changes to too big of an area of code.  If a class or function is medium to large in size and new helper classes are required, then the task should be size such that only one additional simple helper class is created and the remainder of the code can be left untouched for this task.
5. Think and create a step by step high level plan for simplifying the code area, make sure you use the simplifying-code and developing-csharp-code skills for this task.  Apply best practises outlined in these skills.
6. Keep interating on the code until a suitable level of simplification has been achieved.  Code should be analysed for each best practise.

