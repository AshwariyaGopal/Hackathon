<!-- Sync Impact Report:
Version change: None → 1.0.0
List of modified principles:
- Added Technical Accuracy
- Added Clear Explanations
- Added Reproducibility
- Added Consistent Terminology and Tone
- Added Book Standards Adherence
- Added RAG Chatbot Standards Adherence
- Added Citations and Plagiarism
Added sections: Constraints, Success Criteria
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .specify/templates/commands/*.md ⚠ pending
- README.md ⚠ pending
Follow-up TODOs: RATIFICATION_DATE
-->
# Physical AI & Humanoid Robotics — Book + RAG Chatbot Constitution

## Core Principles

### I. Technical Accuracy
All content and code must be technically accurate, specifically concerning ROS 2, Gazebo, Unity, Isaac, VLA, and RAG implementations.

### II. Clear Explanations
Explanations must be clear, step-by-step, and tailored for intermediate AI/robotics learners.

### III. Reproducibility
Code and tutorials must be reproducible, allowing users to replicate results easily.

### IV. Consistent Terminology and Engineering Tone
Maintain consistent terminology and an engineering tone throughout all content.

### V. Book Standards Adherence
The book must adhere to the specified standards: 8 core chapters (ROS2, Gazebo/Unity, Isaac, Nav2/SLAM, URDF, Sensors, VLA, Capstone), 30k–50k words, 20+ runnable code examples, use diagrams when needed, and a Flesch-Kincaid grade of 9–11.

### VI. RAG Chatbot Standards Adherence
The RAG chatbot must strictly answer from book content, include a “selected-text only” mode, use a stack of FastAPI + OpenAI Agents/ChatKit + Qdrant + Neon Postgres, and ensure clean chunking & deterministic retrieval.

### VII. Citations and Plagiarism
All citations must follow IEEE style, include a minimum of 30 credible sources (ROS docs, NVIDIA docs, robotics papers), and maintain a zero-plagiarism tolerance.

## Constraints

The project must be built using Spec-Kit Plus + Claude Code. Final deployment will be on Docusaurus on GitHub Pages. The chatbot must be fully embedded and functional.

## Success Criteria

The book must build and deploy successfully. The chatbot must answer accurately. All content must be verifiable and runnable.

## Governance

Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown. | **Last Amended**: 2025-12-07
