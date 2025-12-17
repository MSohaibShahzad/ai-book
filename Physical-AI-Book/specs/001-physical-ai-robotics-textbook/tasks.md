# Tasks: Physical-AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, data-model.md, research.md, contracts/, quickstart.md

**Tests**: No automated tests requested for this documentation project. Validation through manual review, frontmatter schema validation, and link checking.

**Organization**: Tasks are grouped by user story (module) to enable independent implementation and testing of each module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Module 1, US2=Module 2, US3=Module 3, US4=Module 4)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project root**: `textbook/`
- **Content files**: `textbook/docs/`
- **Diagrams**: `textbook/docs/diagrams/`
- **Configuration**: `textbook/docusaurus.config.js`, `textbook/sidebars.js`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and create foundational structure

- [X] T001 Create textbook project directory structure per plan.md (textbook/, docs/, src/, static/)
- [X] T002 Initialize Docusaurus 3.x project with Node.js 18+ in textbook/ directory
- [X] T003 [P] Configure docusaurus.config.js with site metadata, navbar, and footer
- [X] T004 [P] Create package.json with Docusaurus 3.x dependencies and React 18+
- [X] T005 [P] Create README.md in textbook/ with project overview and setup instructions
- [X] T006 [P] Create docs/diagrams/ directory for shared SVG diagrams
- [X] T007 [P] Set up validation tools (ajv-cli for frontmatter, markdownlint-cli, markdown-link-check) in package.json

**Checkpoint**: Docusaurus project initialized - module content can now be created

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create shared components that ALL modules depend on

**⚠️ CRITICAL**: No module content can begin until this phase is complete

- [X] T008 Create 00-preface/index.md with textbook overview, learning philosophy, and structure
- [X] T009 [P] Create 00-preface/how-to-use-this-book.md with navigation guide and module sequence
- [X] T010 [P] Create 00-preface/prerequisites.md with required background knowledge (Python, basic robotics)
- [X] T011 [P] Create 00-preface/ethics-and-safety.md with robotics ethics framework and safety considerations
- [X] T012 [P] Create 99-appendices/glossary.md with robotics terminology definitions
- [X] T013 [P] Create 99-appendices/references.md with academic citations and ROS 2 documentation links
- [X] T014 [P] Create 99-appendices/further-reading.md with recommended resources
- [X] T015 Configure sidebars.js with navigation structure for all modules (00-preface → 01-04 modules → 99-appendices)
- [X] T016 Validate frontmatter schema against contracts/module-structure.schema.json for preface and appendices files

**Checkpoint**: Foundation ready - module implementation can now begin in parallel

---

## Phase 3: User Story 1 - Understanding Robotic Middleware Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Students can explain ROS 2 as middleware, diagram node communication, understand URDF, and identify use cases for topics/services/actions

**Independent Test**: A student can explain in their own words what ROS 2 is, why nodes/topics/services exist, and why URDF matters for humanoid robots. They can diagram a simple node communication flow without writing code.

### Module 1 Setup & Structure

- [ ] T017 [P] [US1] Create 01-foundations-ros2/ directory and .answers/ subdirectory
- [ ] T018 [US1] Create 01-foundations-ros2/index.md with module overview, 5 learning outcomes, and prerequisites

### Module 1 Chapters

- [ ] T019 [P] [US1] Create 01-foundations-ros2/01-what-is-ros2.md explaining ROS 2 purpose, middleware concept, and "nervous system" analogy
- [ ] T020 [P] [US1] Create 01-foundations-ros2/02-nodes-topics-services.md explaining publish-subscribe (topics) and request-response (services) patterns
- [ ] T021 [P] [US1] Create 01-foundations-ros2/03-actions-and-parameters.md explaining long-running tasks (actions) and configuration (parameters)
- [ ] T022 [P] [US1] Create 01-foundations-ros2/04-urdf-and-robot-models.md explaining URDF structure, joints, links, and humanoid robot modeling
- [ ] T023 [P] [US1] Create 01-foundations-ros2/05-bridging-ai-agents-to-ros.md explaining Python AI agents integration with ROS 2 controllers

### Module 1 Diagrams

- [ ] T024 [P] [US1] Create docs/diagrams/ros2-node-communication.svg (system architecture: 3 nodes with topics/services/actions)
- [ ] T025 [US1] Validate ros2-node-communication.svg uses grayscale, includes alt text, and references in chapters 01, 02

### Module 1 Examples & Exercises

- [ ] T026 [US1] Create 01-foundations-ros2/examples.md with 3-5 annotated Python pseudocode examples (rclpy publisher, subscriber, service, action client)
- [ ] T027 [US1] Create 01-foundations-ros2/exercises.md with 15 exercises (3 per chapter: Recall, Application, Synthesis)
- [ ] T028 [US1] Create 01-foundations-ros2/.answers/exercises-answers.md with complete instructor solutions, explanations, and grading rubrics for all 15 exercises

### Module 1 Validation

- [ ] T029 [US1] Validate all Module 1 frontmatter against contracts/module-structure.schema.json
- [ ] T030 [US1] Run link checker on all Module 1 files to verify diagram references and internal links
- [ ] T031 [US1] Update sidebars.js to include Module 1 navigation (index, 5 chapters, examples, exercises)

**Checkpoint**: Module 1 (ROS 2) complete and independently testable. Students can explain middleware fundamentals.

---

## Phase 4: User Story 2 - Grasping Digital Twin Simulation (Priority: P2)

**Goal**: Students can articulate the purpose of digital twins, compare Gazebo and Unity use cases, and explain why simulating sensors matters for perception algorithms

**Independent Test**: A student can articulate the purpose of digital twins, compare Gazebo and Unity use cases, and explain why simulating sensors (LiDAR, depth cameras, IMU) is necessary for testing perception algorithms.

### Module 2 Setup & Structure

- [ ] T032 [P] [US2] Create 02-digital-twin/ directory and .answers/ subdirectory
- [ ] T033 [US2] Create 02-digital-twin/index.md with module overview, 5 learning outcomes, and prerequisites (references Module 1 ROS 2 knowledge)

### Module 2 Chapters

- [ ] T034 [P] [US2] Create 02-digital-twin/01-why-simulation-matters.md explaining safety, cost-effectiveness, and rapid iteration benefits
- [ ] T035 [P] [US2] Create 02-digital-twin/02-digital-twins-concept.md explaining virtual representations and physics-based simulation
- [ ] T036 [P] [US2] Create 02-digital-twin/03-gazebo-physics-simulation.md explaining Gazebo's physics engines, sensors, and ROS 2 integration
- [ ] T037 [P] [US2] Create 02-digital-twin/04-unity-high-fidelity-rendering.md explaining Unity for HRI visualization and photorealistic rendering
- [ ] T038 [P] [US2] Create 02-digital-twin/05-sensor-simulation.md explaining LiDAR, depth cameras, IMU simulation for perception testing

### Module 2 Diagrams

- [ ] T039 [P] [US2] Create docs/diagrams/digital-twin-architecture.svg (conceptual diagram: physical robot ↔ digital twin ↔ simulation)
- [ ] T040 [US2] Validate digital-twin-architecture.svg uses grayscale, includes alt text, and references in chapters 01, 02, 03

### Module 2 Examples & Exercises

- [ ] T041 [US2] Create 02-digital-twin/examples.md with 3-5 examples (Gazebo world file snippets, Unity scene concepts, sensor plugin configurations)
- [ ] T042 [US2] Create 02-digital-twin/exercises.md with 15 exercises (3 per chapter: Recall, Application, Synthesis)
- [ ] T043 [US2] Create 02-digital-twin/.answers/exercises-answers.md with complete instructor solutions, explanations, and grading rubrics

### Module 2 Validation

- [ ] T044 [US2] Validate all Module 2 frontmatter against contracts/module-structure.schema.json
- [ ] T045 [US2] Run link checker on all Module 2 files to verify diagram references and cross-module links to Module 1
- [ ] T046 [US2] Update sidebars.js to include Module 2 navigation

**Checkpoint**: Module 2 (Digital Twin) complete and independently testable. Students understand simulation trade-offs.

---

## Phase 5: User Story 3 - Comprehending GPU-Accelerated Perception (Priority: P3)

**Goal**: Students can explain the value proposition of Isaac Sim for photorealistic environments, why synthetic data matters for perception model training, and how Isaac ROS accelerates VSLAM and navigation

**Independent Test**: A student can explain the value proposition of Isaac Sim for photorealistic environments, why synthetic data matters for perception model training, and how Isaac ROS accelerates VSLAM and navigation tasks.

### Module 3 Setup & Structure

- [ ] T047 [P] [US3] Create 03-ai-robot-brain/ directory and .answers/ subdirectory
- [ ] T048 [US3] Create 03-ai-robot-brain/index.md with module overview, 5 learning outcomes, and prerequisites (references Modules 1 and 2)

### Module 3 Chapters

- [ ] T049 [P] [US3] Create 03-ai-robot-brain/01-gpu-accelerated-simulation.md explaining GPU advantages for photorealistic rendering and physics
- [ ] T050 [P] [US3] Create 03-ai-robot-brain/02-isaac-sim-photorealism.md explaining Isaac Sim's ray tracing, material simulation, and realism benefits
- [ ] T051 [P] [US3] Create 03-ai-robot-brain/03-synthetic-data-generation.md explaining labeled data generation for perception model training
- [ ] T052 [P] [US3] Create 03-ai-robot-brain/04-isaac-ros-perception.md explaining Isaac ROS GEMs (GPU-accelerated perception nodes)
- [ ] T053 [P] [US3] Create 03-ai-robot-brain/05-vslam-and-nav2-integration.md explaining Visual SLAM, Nav2 stack, and bipedal locomotion navigation

### Module 3 Diagrams

- [ ] T054 [P] [US3] Create docs/diagrams/isaac-sim-pipeline.svg (process flow: scene setup → synthetic data → perception training → Isaac ROS deployment)
- [ ] T055 [US3] Validate isaac-sim-pipeline.svg uses grayscale, includes alt text, and references in chapters 02, 03, 04

### Module 3 Examples & Exercises

- [ ] T056 [US3] Create 03-ai-robot-brain/examples.md with 3-5 examples (Isaac Sim scene concepts, synthetic data annotation examples, Isaac ROS node configurations)
- [ ] T057 [US3] Create 03-ai-robot-brain/exercises.md with 15 exercises (3 per chapter: Recall, Application, Synthesis)
- [ ] T058 [US3] Create 03-ai-robot-brain/.answers/exercises-answers.md with complete instructor solutions, explanations, and grading rubrics

### Module 3 Validation

- [ ] T059 [US3] Validate all Module 3 frontmatter against contracts/module-structure.schema.json
- [ ] T060 [US3] Run link checker on all Module 3 files to verify diagram references and cross-module links
- [ ] T061 [US3] Update sidebars.js to include Module 3 navigation

**Checkpoint**: Module 3 (Isaac) complete and independently testable. Students understand GPU-accelerated perception.

---

## Phase 6: User Story 4 - Understanding Vision-Language-Action Pipelines (Priority: P4)

**Goal**: Students can diagram a voice-to-action pipeline (Whisper → LLM → ROS 2 actions), explain why LLMs enable cognitive planning, and describe how a capstone humanoid project integrates voice, navigation, and manipulation

**Independent Test**: A student can diagram a voice-to-action pipeline (Whisper → LLM → ROS 2 actions), explain why LLMs enable cognitive planning, and describe how a capstone humanoid project integrates voice, navigation, and manipulation.

### Module 4 Setup & Structure

- [ ] T062 [P] [US4] Create 04-vision-language-action/ directory and .answers/ subdirectory
- [ ] T063 [US4] Create 04-vision-language-action/index.md with module overview, 5 learning outcomes, and prerequisites (references all prior modules)

### Module 4 Chapters

- [ ] T064 [P] [US4] Create 04-vision-language-action/01-voice-to-action-pipelines.md explaining speech → text → intent → action flow
- [ ] T065 [P] [US4] Create 04-vision-language-action/02-whisper-speech-recognition.md explaining Whisper architecture and voice transcription concepts
- [ ] T066 [P] [US4] Create 04-vision-language-action/03-llms-for-cognitive-planning.md explaining how LLMs decompose high-level commands into sub-tasks
- [ ] T067 [P] [US4] Create 04-vision-language-action/04-llm-to-ros2-action-generation.md explaining translation from LLM outputs to ROS 2 service/action calls
- [ ] T068 [P] [US4] Create 04-vision-language-action/05-capstone-humanoid-autonomy.md with integrated capstone project (voice command → VLA reasoning → navigation → manipulation)

### Module 4 Diagrams

- [ ] T069 [P] [US4] Create docs/diagrams/vla-pipeline.svg (process flow: voice input → Whisper → LLM → ROS 2 actions → robot execution)
- [ ] T070 [P] [US4] Create docs/diagrams/capstone-integration-flow.svg (system architecture: all 4 modules integrated in humanoid autonomy scenario)
- [ ] T071 [US4] Validate vla-pipeline.svg and capstone-integration-flow.svg use grayscale, include alt text, and reference in appropriate chapters

### Module 4 Examples & Exercises

- [ ] T072 [US4] Create 04-vision-language-action/examples.md with 3-5 examples (Whisper API concepts, LLM prompting for robotics, ROS 2 action generation pseudocode)
- [ ] T073 [US4] Create 04-vision-language-action/exercises.md with 15 exercises (3 per chapter: Recall, Application, Synthesis)
- [ ] T074 [US4] Create 04-vision-language-action/.answers/exercises-answers.md with complete instructor solutions, explanations, and grading rubrics

### Module 4 Validation

- [ ] T075 [US4] Validate all Module 4 frontmatter against contracts/module-structure.schema.json
- [ ] T076 [US4] Run link checker on all Module 4 files to verify diagram references and cross-module links
- [ ] T077 [US4] Update sidebars.js to include Module 4 navigation

**Checkpoint**: Module 4 (VLA) complete and independently testable. All user stories satisfied. Capstone project integrates all modules.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validations that affect the entire textbook

- [ ] T078 [P] Create textbook/static/img/ directory and add textbook logo and cover image
- [ ] T079 [P] Review all markdown files for consistent academic tone, clear explanations, and WHY-before-HOW approach
- [ ] T080 [P] Validate all YAML frontmatter across all files using ajv-cli and contracts/module-structure.schema.json
- [ ] T081 [P] Run markdownlint-cli on all markdown files to ensure consistent formatting
- [ ] T082 Run markdown-link-check on all files to verify no broken internal or external links
- [ ] T083 Verify all diagrams (5 SVG files) exist in docs/diagrams/, use grayscale, and include alt text
- [ ] T084 [P] Test Docusaurus build process (npm run build) and verify static site generation succeeds in under 2 minutes
- [ ] T085 [P] Test local development server (npm start) and manually navigate through all modules to verify sidebar, search, and navigation
- [ ] T086 Validate that .answers/ directories are excluded from sidebar configuration (instructor solutions hidden from students)
- [ ] T087 [P] Create textbook/LICENSE file with CC BY-NC-SA 4.0 International License text
- [ ] T088 [P] Review glossary (99-appendices/glossary.md) for completeness of all technical terms used across modules
- [ ] T089 [P] Review references (99-appendices/references.md) for accurate citations and up-to-date links
- [ ] T090 Run full validation suite from quickstart.md (frontmatter validation, link checking, markdown linting)
- [ ] T091 Generate PDF export using Docusaurus PDF plugin or Pandoc and verify formatting, diagrams, and table of contents
- [ ] T092 Review ethics-and-safety.md (00-preface/) to ensure coverage of robotics ethics framework and safety warnings
- [ ] T093 Final review of all 4 module indexes to verify learning outcomes, prerequisites, and references are complete

**Checkpoint**: Textbook complete, validated, and ready for deployment or peer review

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all modules
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - Modules can then proceed in parallel (if multiple contributors)
  - Or sequentially in priority order (Module 1 → 2 → 3 → 4)
- **Polish (Phase 7)**: Depends on all modules being complete

### User Story Dependencies

- **User Story 1 / Module 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other modules (self-contained ROS 2 foundations)
- **User Story 2 / Module 2 (P2)**: Can start after Foundational (Phase 2) - References Module 1 concepts but independently testable
- **User Story 3 / Module 3 (P3)**: Can start after Foundational (Phase 2) - References Modules 1 and 2 but independently testable
- **User Story 4 / Module 4 (P4)**: Can start after Foundational (Phase 2) - Synthesizes all prior modules but independently testable with capstone project

### Within Each Module

- Module directory creation before content files
- Index file before chapters
- All chapters can be written in parallel [P]
- Diagrams can be created in parallel with chapters [P]
- Examples after chapters (may reference chapter content)
- Exercises after examples (may reference both chapters and examples)
- Instructor solutions (.answers/) after exercises
- Frontmatter validation after all content is written
- Sidebar update after validation

### Parallel Opportunities

**Phase 1 (Setup)**: T003, T004, T005, T006, T007 can run in parallel

**Phase 2 (Foundational)**: T009, T010, T011, T012, T013, T014 can run in parallel after T008

**Within Each Module**:
- All chapter files can be created in parallel [P]
- All diagram files can be created in parallel [P]
- Frontmatter validation and link checking can run in parallel after content creation

**Across Modules** (if team capacity allows):
- After Foundational phase completes, all 4 modules (US1-US4) can be worked on in parallel by different contributors
- Each module is independently testable and deliverable

**Phase 7 (Polish)**: T078, T079, T080, T081, T084, T085, T087, T088, T089 can run in parallel

---

## Parallel Example: Module 1 (User Story 1)

```bash
# After T017 (directory creation) and T018 (index.md) complete, launch all chapters in parallel:
Task T019: "Create 01-foundations-ros2/01-what-is-ros2.md"
Task T020: "Create 01-foundations-ros2/02-nodes-topics-services.md"
Task T021: "Create 01-foundations-ros2/03-actions-and-parameters.md"
Task T022: "Create 01-foundations-ros2/04-urdf-and-robot-models.md"
Task T023: "Create 01-foundations-ros2/05-bridging-ai-agents-to-ros.md"

# Simultaneously, create diagrams in parallel:
Task T024: "Create docs/diagrams/ros2-node-communication.svg"

# After chapters complete, create examples, exercises, and solutions sequentially:
Task T026: "Create 01-foundations-ros2/examples.md"
Task T027: "Create 01-foundations-ros2/exercises.md"
Task T028: "Create 01-foundations-ros2/.answers/exercises-answers.md"
```

---

## Implementation Strategy

### MVP First (Module 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all modules)
3. Complete Phase 3: Module 1 (ROS 2 Middleware Fundamentals)
4. **STOP and VALIDATE**: Test Module 1 independently - students can explain ROS 2, diagram nodes, understand URDF
5. Deploy/demo Module 1 as standalone learning resource if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Module 1 → Test independently → Deploy/Demo (MVP - ROS 2 foundations)
3. Add Module 2 → Test independently → Deploy/Demo (MVP + Digital Twin simulation)
4. Add Module 3 → Test independently → Deploy/Demo (MVP + Isaac perception)
5. Add Module 4 → Test independently → Deploy/Demo (Complete textbook with capstone)
6. Each module adds value without breaking previous modules

### Parallel Team Strategy

With multiple contributors (e.g., 4 content authors):

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Module 1 (ROS 2)
   - Author B: Module 2 (Digital Twin)
   - Author C: Module 3 (Isaac)
   - Author D: Module 4 (VLA)
3. Modules complete and integrate independently via shared diagrams folder and cross-references

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story (module) for traceability
- Each module (user story) should be independently completable and testable
- No automated tests for documentation project - validation through schema validation, link checking, and manual review
- Commit after each task or logical group (e.g., all chapters in a module)
- Stop at any checkpoint to validate module independently
- Follow constitution principles: WHY-before-HOW, academic tone, kebab-case filenames, YAML frontmatter
- Diagrams must be SVG, grayscale, with alt text for accessibility
- Exercises must follow 3-tier Bloom's Taxonomy (Recall, Application, Synthesis)
- Instructor solutions (.answers/) must be excluded from student-facing sidebar
- All frontmatter must validate against contracts/module-structure.schema.json

---

## Task Count Summary

- **Total Tasks**: 93
- **Setup (Phase 1)**: 7 tasks
- **Foundational (Phase 2)**: 9 tasks (CRITICAL - blocks all modules)
- **Module 1 / US1 (Phase 3)**: 15 tasks
- **Module 2 / US2 (Phase 4)**: 15 tasks
- **Module 3 / US3 (Phase 5)**: 15 tasks
- **Module 4 / US4 (Phase 6)**: 16 tasks
- **Polish (Phase 7)**: 16 tasks

**Parallel Opportunities**: 42 tasks marked [P] can run in parallel within their respective phases

**Independent Test Criteria**:
- **US1/Module 1**: Student explains ROS 2 middleware, diagrams node communication
- **US2/Module 2**: Student compares Gazebo vs Unity, explains sensor simulation value
- **US3/Module 3**: Student explains Isaac Sim value, synthetic data benefits, VSLAM concepts
- **US4/Module 4**: Student diagrams VLA pipeline, traces capstone integration flow

**Suggested MVP Scope**: Phases 1 + 2 + 3 (Setup + Foundational + Module 1) = 31 tasks
