# Content Data Model: Module 1 ROS 2 Basics

## Summary

This document defines the structure and relationships of the content entities for the "Module 1: ROS 2 — The Robotic Nervous System" feature. As this is a documentation project, the "data model" refers to the organization of the educational content itself, not a database schema.

## Content Entities

### 1. Chapter

The primary container for a distinct learning topic.

-   **Fields**:
    -   `title` (string): The official name of the chapter (e.g., "ROS 2 Communication: Nodes, Topics, Services").
    -   `learning_outcomes` (list of strings): A bulleted list of skills or knowledge a student will gain.
    -   `body` (Markdown): The main instructional text for the chapter.
    -   `word_count` (integer): The total number of words in the `body`.

-   **Validation Rules**:
    -   `title` is mandatory and must be unique within the module.
    -   `learning_outcomes` must contain at least one item.
    -   The module as a whole must have a total word count between 3,000 and 6,000 words.

### 2. Code Example

A runnable snippet of code used to demonstrate a concept.

-   **Fields**:
    -   `name` (string): The filename for the code snippet (e.g., `simple_publisher.py`).
    -   `language` (string): The programming language used. Will be `Python` for this module.
    -   `dependencies` (list of strings): A list of required libraries (e.g., `rclpy`).
    -   `purpose` (string): A brief description of what the example demonstrates.

-   **Relationships**:
    -   A `Chapter` contains one or more `Code Examples`. Each example is embedded and explained within the chapter's `body`.
    -   All `Code Examples` are collected into a single, buildable ROS 2 package located in the `/examples` directory.

### 3. Diagram

A visual aid to help explain a concept.

-   **Fields**:
    -   `name` (string): The filename for the diagram image (e.g., `topic_communication.png`).
    -   `format` (string): The image file type (e.g., PNG, SVG).
    -   `caption` (string): A brief, descriptive text that appears below the diagram.
    -   `alt_text` (string): A description of the image for accessibility.

-   **Relationships**:
    -   A `Chapter` can contain zero or more `Diagrams`. Each diagram is embedded within the chapter's `body`.
    -   All `Diagrams` are stored in the `docs/assets/images` directory.

### 4. URDF Model

A file defining the structure of a robot.

-   **Fields**:
    -   `name` (string): The filename for the model (e.g., `simple_humanoid.urdf`).
    -   `root_link` (string): The name of the base link of the robot model.

-   **Relationships**:
    -   The `Chapter` on URDF contains one `URDF Model`.
    -   The `URDF Model` is used as a target for control commands in the `rclpy` chapter's `Code Examples`.
