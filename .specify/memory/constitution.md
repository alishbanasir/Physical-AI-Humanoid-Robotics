<!--
Sync Impact Report:
- Version Change: [unversioned] → 1.0.0
- Modified Principles: Initial establishment of all principles
- Added Sections: Core Principles (6), Content Standards, Technical Architecture, Development Workflow, Governance
- Templates Requiring Updates: ✅ Constitution created (first version)
- Follow-up TODOs: Verify template alignment after first feature spec is created
- Rationale: MAJOR version 1.0.0 for initial constitution establishment
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy & Depth
Content MUST be technically accurate, comprehensive, and aligned with professional robotics engineering standards. Every chapter requires:
- Detailed theoretical foundations with mathematical formulations where applicable
- Practical implementation examples using industry-standard tools (ROS 2, Gazebo, Unity, NVIDIA Isaac)
- Code samples that are executable, well-commented, and follow best practices
- Clear explanations of complex concepts accessible to students while maintaining technical rigor

**Rationale**: As an educational resource for Physical AI and Humanoid Robotics, superficial or inaccurate content undermines learning outcomes and professional credibility.

### II. Modular Structure Integrity (NON-NEGOTIABLE)
The textbook architecture is fixed at 4 Modules × 4 Chapters = 16 Chapters total:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)

**Rationale**: This structure reflects the pedagogical progression from foundational systems (ROS 2) through simulation (Digital Twin) to AI integration (NVIDIA Isaac) and culminates in cutting-edge VLA models. Deviation disrupts the learning pathway.

### III. AI-Native Development & Spec-Driven Approach
All development MUST follow Spec-Driven Development (SDD) methodology:
- Every feature begins with a detailed spec (spec.md)
- Architecture decisions documented in plan (plan.md)
- Implementation broken into testable tasks (tasks.md)
- Claude Subagents utilized for bonus points where appropriate
- Prompt History Records (PHRs) created for every significant development step

**Rationale**: AI-native methodology ensures reproducibility, maintainability, and alignment with hackathon bonus criteria (Claude Subagents).

### IV. Professional UI/UX Standards
The Docusaurus-based textbook MUST exhibit professional-grade design:
- Custom theme with cohesive color palette, typography, and spacing
- Responsive design optimized for desktop, tablet, and mobile
- Accessible navigation (sidebar, search, breadcrumbs)
- Syntax-highlighted code blocks with copy functionality
- Mathematical notation rendered via KaTeX or similar
- Visual hierarchy distinguishing headings, callouts, examples, and exercises

**Rationale**: A polished, attractive UI enhances engagement and reflects the quality standards of institutions like Panaversity. Default Docusaurus themes are insufficient for differentiation.

### V. Free-Tier Infrastructure Optimization
All backend services (RAG chatbot, authentication, database) MUST operate within free-tier limits:
- Qdrant Cloud Free Tier for vector storage
- Neon Serverless Postgres Free Tier for user data
- OpenAI API usage minimized through efficient chunking and caching
- FastAPI backend designed for low resource consumption
- Vercel/GitHub Pages deployment optimization

**Rationale**: Budget constraints and hackathon evaluation criteria require demonstrating efficiency. Scalability considerations are secondary to immediate viability.

### VI. RAG System Content Fidelity
The RAG chatbot MUST answer questions based exclusively on textbook content:
- Vector embeddings generated from finalized chapter markdown
- Retrieval mechanisms prevent hallucination or external knowledge injection
- Responses cite specific chapters/sections when applicable
- System prompts enforce strict grounding in retrieved context
- Regular validation against ground-truth Q&A pairs from textbook content

**Rationale**: Educational integrity demands that the AI assistant reinforces textbook material rather than introducing external or conflicting information.

## Content Standards

### Chapter Requirements
Each of the 16 chapters MUST include:
1. **Learning Objectives**: 3-5 concrete, measurable outcomes
2. **Theoretical Foundations**: Core concepts, algorithms, mathematical models
3. **Hands-On Implementation**: Step-by-step tutorials with code (ROS 2, Python, C++)
4. **Practical Examples**: Real-world applications and case studies
5. **Exercises & Challenges**: Progressive difficulty (beginner to advanced)
6. **Further Reading**: Curated references to papers, documentation, and resources

### Code Quality Standards
All code samples MUST:
- Execute without errors in specified environments (ROS 2 Humble/Jazzy, Ubuntu 22.04/24.04)
- Include setup instructions and dependency lists
- Follow language-specific style guides (PEP 8 for Python, ROS 2 conventions)
- Contain inline comments explaining non-obvious logic
- Be tested in isolated environments before publication

### Content Review Gates
Before a chapter is marked complete:
- Technical accuracy verified against official documentation (ROS 2, NVIDIA Isaac, Unity)
- Code samples tested in clean environment
- Markdown formatting validated (no broken links, images render correctly)
- Learning objectives mapped to chapter content
- Readability checked (Flesch-Kincaid grade level appropriate for undergraduate/graduate students)

## Technical Architecture

### Docusaurus Configuration
- **Version**: Docusaurus 3.x (latest stable)
- **Theme**: Custom theme extending @docusaurus/theme-classic
- **Plugins**: docs, blog (optional), search (Algolia or local)
- **Markdown**: MDX with support for React components, KaTeX, Mermaid diagrams

### RAG Chatbot Stack
- **Vector Database**: Qdrant (cloud free tier, 1GB storage, 100K vectors)
- **Relational Database**: Neon Serverless Postgres (free tier, 512MB storage)
- **Backend**: FastAPI (Python 3.10+)
- **Embeddings**: OpenAI text-embedding-3-small (lower cost than ada-002)
- **LLM**: OpenAI GPT-4o-mini (cost-efficient for educational Q&A)
- **Frontend Integration**: React components embedded in Docusaurus

### Authentication & Personalization
- **Auth Provider**: Better-Auth (modern, type-safe, self-hosted)
- **User Database**: Neon Postgres (user profiles, preferences)
- **Features**:
  - Signup/Signin with email/password
  - OAuth (GitHub, Google) - optional stretch goal
  - User preferences: theme (light/dark), language (English/Urdu)
  - Personalized learning progress tracking (chapters completed)

### Translation (Urdu Support)
- **Approach**: Docusaurus i18n (Internationalization) plugin
- **Scope**: UI labels and selected chapters (priority: Module 1)
- **Quality**: Native Urdu speakers review translations for technical accuracy
- **Toggle**: Language switcher in navbar, persisted in user preferences if logged in

## Development Workflow

### Feature Development Lifecycle
1. **Specification** (`/sp.specify`): Define feature requirements, success criteria, constraints
2. **Planning** (`/sp.plan`): Architect solution, identify components, dependencies, ADR candidates
3. **Task Generation** (`/sp.tasks`): Break plan into testable, atomic tasks with acceptance criteria
4. **Implementation** (`/sp.implement`): Execute tasks, run tests, validate acceptance criteria
5. **Review**: Manual QA, cross-browser testing, accessibility checks
6. **Deployment**: Commit to main, trigger Vercel deployment, verify production

### Git Workflow
- **Branch Strategy**: Feature branches off `main` (e.g., `feature/rag-chatbot`, `feature/module-1-chapter-1`)
- **Commit Messages**: Conventional Commits format (`feat:`, `fix:`, `docs:`, `chore:`)
- **Pull Requests**: Required for merging to `main`, must pass CI checks (build, lint)
- **Tags**: Version tags for major milestones (e.g., `v1.0.0` for hackathon submission)

### Quality Gates
Before merging any feature:
- [ ] Docusaurus builds without errors (`npm run build`)
- [ ] All internal links resolve correctly
- [ ] Code samples execute in test environment
- [ ] Responsive design verified (desktop, tablet, mobile)
- [ ] Accessibility: WCAG 2.1 AA compliance (headings, alt text, keyboard navigation)
- [ ] PHR created documenting implementation

### Testing Strategy
- **Content Validation**: Automated checks for broken links, missing images, invalid markdown
- **RAG System**: Q&A accuracy against ground-truth dataset (minimum 90% relevance)
- **Authentication**: End-to-end tests for signup, login, logout, session persistence
- **UI Components**: Visual regression testing (Percy or Chromatic - if budget allows, else manual)

## Governance

### Amendment Process
1. Propose change in GitHub issue or discussion, tagged `constitution-amendment`
2. Document rationale, affected principles, and downstream impact
3. Update constitution with version bump (MAJOR/MINOR/PATCH per semantic versioning)
4. Update dependent templates (spec, plan, tasks) to reflect changes
5. Create ADR if amendment affects architectural decisions
6. Commit with message: `docs: amend constitution to vX.Y.Z (<brief>)`

### Versioning Policy
- **MAJOR** (X.0.0): Removal or redefinition of core principles, breaking changes to development workflow
- **MINOR** (x.Y.0): Addition of new principles, sections, or substantial expansions
- **PATCH** (x.y.Z): Clarifications, typo fixes, formatting improvements, non-semantic edits

### Compliance & Enforcement
- All specs, plans, and tasks MUST reference applicable constitution principles
- Code reviews verify adherence to content standards and technical architecture
- Deviations from free-tier optimization or modular structure require explicit justification and approval
- Hackathon submission checklist cross-references constitution compliance

### Living Document Status
This constitution is authoritative for the duration of the hackathon and initial deployment. Post-hackathon iterations may relax constraints (e.g., free-tier limits if funding secured) but must follow amendment process.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
