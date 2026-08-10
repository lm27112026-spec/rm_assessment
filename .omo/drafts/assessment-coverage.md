# Draft: Assessment Coverage

## Requirements (confirmed)
- User wants to know which of the assignment requirements are already implemented in the current codebase.
- Scope includes题目1-4 and bonus items.
- Need evidence from existing files only; no code changes.
- User clarified that `learning/` is reference code only and must NOT be counted as actual implementation.
- New request: create a work plan to implement 题目3 using a YOLOv5 model, based on the reference code under `learning/`, and place the new implementation in the appropriate actual-code folder.

## Technical Decisions
- `learning/` may be used as reference material but should not be treated as production implementation.
- Need to plan production YOLOv5 module placement under root-level actual project structure, likely outside `learning/`.

## Research Findings
- Launched read-only exploration to map the YOLOv5 reference implementation, test infrastructure, and root CMake layout before finalizing the plan.

## Open Questions
- Which requirements are fully implemented, partially implemented, or not yet implemented?
- Where should the production YOLOv5 implementation live if multiple valid layouts are possible?
- Should automated tests be TDD, tests-after, or omitted for YOLOv5 work?

## Scope Boundaries
- INCLUDE: requirement coverage analysis of the repository.
- INCLUDE as actual code: root-level `io/`, `src/`, `tasks/`, `tests/`, root `CMakeLists.txt`.
- EXCLUDE from actual implementation credit: `learning/` reference code.
- INCLUDE for new plan: use `learning/` as read-only reference for YOLOv5 implementation.
- EXCLUDE: code modification, implementation, file creation outside this draft.
