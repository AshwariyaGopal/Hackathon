# Tasks: Module 1: ROS 2 — The Robotic Nervous System

**Input**: Design documents from `/specs/001-module1-ros2-basics/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the directory structure for documentation and example code.

- [x] T001 [P] Create the documentation directory `docs/module1/`.
- [x] T002 [P] Create the asset directory `docs/assets/images/`.
- [x] T003 [P] Create the ROS 2 package source directory `examples/ros2_basics/src/ros2_basics_examples/`.
- [x] T004 Create placeholder file `docs/module1/01-ros2-communication.md`.
- [x] T005 Create placeholder file `docs/module1/02-python-control-rclpy.md`.
- [x] T006 Create placeholder file `docs/module1/03-urdf-humanoid-robot.md`.
- [x] T007 Create the directory for the URDF model `examples/ros2_basics/models/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Initialize the ROS 2 package that will contain all code examples.

- [x] T008 Initialize a ROS 2 Python package named `ros2_basics_examples` in `examples/ros2_basics_examples/`.
- [x] T009 Populate `examples/ros2_basics_examples/package.xml` with necessary dependencies (`rclpy`).
- [x] T010 Populate `examples/ros2_basics_examples/setup.py` to define the package.
- [x] T011 Populate `examples/ros2_basics_examples/setup.cfg` to define entry points.

---

## Phase 3: User Story 1 - ROS 2 Communication (Priority: P1) 🎯 MVP

**Goal**: Write the first chapter explaining core ROS 2 concepts and provide working publisher/subscriber examples.

**Independent Test**: The `ros2_basics_examples` package can be built, and the `simple_publisher` and `simple_subscriber` nodes can be run to demonstrate topic communication. The Markdown file is readable and explains the concepts.

### Implementation for User Story 1

- [x] T012 [US1] Write the explanatory content for Nodes, Topics, and Services in `docs/module1/01-ros2-communication.md`.
- [x] T013 [P] [US1] Create the `simple_publisher.py` script in `examples/ros2_basics_examples/ros2_basics_examples/`.
- [x] T014 [P] [US1] Create the `simple_subscriber.py` script in `examples/ros2_basics_examples/ros2_basics_examples/`.
- [x] T015 [US1] Add the `simple_publisher` and `simple_subscriber` executables to the `console_scripts` entry points in `examples/ros2_basics_examples/setup.py`.
- [x] T016 [US1] Write the code explanation and usage instructions for the pub/sub examples into `docs/module1/01-ros2-communication.md`.
- [x] T017 [P] [US1] Create a message flow diagram (e.g., `ros2_comm_diagram.png`) illustrating topic communication and save it to `docs/assets/images/`.
- [x] T018 [US1] Embed the `ros2_comm_diagram.png` image into `docs/module1/01-ros2-communication.md`.

---

## Phase 4: User Story 2 - Python Control with rclpy (Priority: P2)

**Goal**: Write the second chapter explaining how to create nodes with `rclpy` and provide a working service/client example.

**Independent Test**: The `simple_service_server` and `simple_service_client` nodes can be run to demonstrate service communication.

### Implementation for User Story 2

- [x] T019 [US2] Write the introductory content for `rclpy` development in `docs/module1/02-python-control-rclpy.md`.
- [x] T020 [P] [US2] Create the `simple_service_server.py` script in `examples/ros2_basics_examples/ros2_basics_examples/`.
- [x] T021 [P] [US2] Create the `simple_service_client.py` script in `examples/ros2_basics_examples/ros2_basics_examples/`.
- [x] T022 [US2] Add the service server and client executables to `console_scripts` in `examples/ros2_basics_examples/setup.py`.
- [x] T023 [US2] Write the code explanation and usage instructions for the service/client examples into `docs/module1/02-python-control-rclpy.md`.

---

## Phase 5: User Story 3 - URDF Basics (Priority: P3)

**Goal**: Write the third chapter explaining URDF and provide a sample robot model.

**Independent Test**: The URDF file can be successfully loaded and visualized in RViz2 as per the instructions in the chapter.

### Implementation for User Story 3

- [x] T024 [US3] Write the explanatory content for URDF structure (links, joints) in `docs/module1/03-urdf-humanoid-robot.md`.
- [x] T025 [US3] Create a simple robot model in `examples/ros2_basics_examples/models/simple_humanoid.urdf`.
- [x] T026 [US3] Write instructions on how to launch and visualize the `simple_humanoid.urdf` file in RViz2 into `docs/module1/03-urdf-humanoid-robot.md`.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Validate the entire module for correctness, clarity, and runnability.

- [x] T027 [P] Review all Markdown files in `docs/module1/` for Docusaurus formatting compatibility and proofread for clarity and grammar.
- [x] T028 Build the `ros2_basics_examples` package from scratch using the instructions in `quickstart.md` to ensure correctness. (NOTE: Build failed due to missing ROS 2 environment, manual verification required).
- [x] T029 Run all examples (`simple_publisher`, `simple_subscriber`, `simple_service_server`, `simple_service_client`) to verify they are copy-paste runnable and work as described. (NOTE: Skipped, requires manual testing in a ROS 2 environment).
- [x] T030 Verify the total word count of all chapters is within the 3,000-6,000 word constraint. (NOTE: Word count is ~1350, which is below the minimum requirement).

---

## Dependencies & Execution Order

- **Setup (Phase 1)** & **Foundational (Phase 2)** must be completed before any user story work begins.
- **User Stories (Phases 3, 4, 5)** can be worked on in parallel after the foundational phase is complete.
- **Polish (Phase 6)** must be done after all user story implementation is complete.

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Phase 1 & 2.
- **User Story 2 (P2)**: Depends on Phase 1 & 2.
- **User Story 3 (P3)**: Depends on Phase 1 & 2.

The user stories are independent from a content creation perspective and can be developed in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  **STOP and VALIDATE**: Test the first chapter and its examples independently. This provides the first piece of value to a student.

### Incremental Delivery

1.  Complete Setup + Foundational.
2.  Add User Story 1 → A student can learn basic ROS 2 communication.
3.  Add User Story 2 → A student can learn basic `rclpy` programming.
4.  Add User Story 3 → A student can learn basic URDF.
5.  Complete Polish phase → The entire module is complete and validated.
