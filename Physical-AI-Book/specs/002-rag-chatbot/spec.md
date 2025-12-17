# Feature Specification: RAG Chatbot for Physical-AI Textbook

**Feature Branch**: `002-rag-chatbot`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "A chatbot embedded in the textbook website that can answer questions about any module or chapter, explain highlighted text, retrieve relevant passages from the textbook, and provide short references (module/chapter)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Global Question Answering (Priority: P1)

A student is studying the Physical-AI textbook and has a question about a concept they don't fully understand. They want to quickly get an answer without manually searching through multiple chapters.

**Why this priority**: This is the core value proposition—enabling students to ask any question about textbook content and receive immediate, grounded answers. This directly addresses the goal of making the textbook interactive.

**Independent Test**: Student opens the chat widget, asks "What is inverse kinematics?", and receives an answer with 2-5 relevant textbook passages cited with module/chapter names.

**Acceptance Scenarios**:

1. **Given** student is viewing any textbook page, **When** they open the floating chat widget and type "What is ROS 2?", **Then** the system retrieves 2-5 relevant passages from the textbook, displays an answer synthesizing the content, and shows source references like "Source: Module 1, Chapter 2"

2. **Given** student asks a multi-part question like "How do forward and inverse kinematics differ?", **When** the system processes the query, **Then** the answer addresses both concepts and cites multiple relevant sections from the textbook

3. **Given** student asks a question about content not covered in the textbook, **When** the retrieval system finds no relevant passages, **Then** the chatbot responds "This topic is not covered in the textbook" without fabricating information

---

### User Story 2 - Highlighted Text Explanation (Priority: P2)

A student encounters a complex paragraph or technical term while reading and wants immediate clarification of that specific content without leaving the page.

**Why this priority**: Context-aware help significantly improves comprehension. This feature differentiates the chatbot from simple search by providing targeted explanations of difficult passages.

**Independent Test**: Student highlights text like "Denavit-Hartenberg parameters", clicks chat widget, asks "explain this", and receives a detailed explanation focusing on the highlighted content with additional context from related sections.

**Acceptance Scenarios**:

1. **Given** student has highlighted a paragraph about neural motor control, **When** they open the chat and ask "what does this mean?", **Then** the system uses the highlighted text as primary context and provides an explanation that references the highlighted section plus 1-2 additional relevant passages

2. **Given** student highlights a code snippet, **When** they ask "how does this work?", **Then** the chatbot explains the code line-by-line using terminology and concepts from the textbook

3. **Given** student highlights mathematical notation or formulas, **When** they request clarification, **Then** the response explains the formula's purpose, variables, and application with examples from the textbook

---

### User Story 3 - Multi-Turn Conversation (Priority: P3)

A student wants to ask follow-up questions about a topic to deepen their understanding within a continuous conversation flow.

**Why this priority**: Enables natural learning progression but is less critical than initial question answering. Provides improved user experience once core functionality is established.

**Independent Test**: Student asks "What is forward kinematics?", receives answer, then follows up with "Can you show me an example?", and the system retrieves relevant code examples from the textbook maintaining conversation context.

**Acceptance Scenarios**:

1. **Given** student has asked a question about ROS 2 topics, **When** they follow up with "give me an example of this", **Then** the chatbot understands "this" refers to ROS 2 topics from previous message and retrieves appropriate examples

2. **Given** an ongoing conversation about a specific module, **When** student asks a related question without repeating the module name, **Then** the system maintains context and prioritizes content from the same module

3. **Given** student closes the chat widget or refreshes the page, **When** they reopen the widget, **Then** the conversation history is cleared and they start a fresh session

---

### Edge Cases

- **What happens when student asks a harmful question (e.g., "how to weaponize a robot arm")?**
  - System refuses to answer and responds: "I cannot provide information on harmful applications. Please refer to the Ethics & Safety section of the textbook."

- **What happens when no relevant passages are found for a query?**
  - System responds: "This topic is not covered in the textbook. Please check the table of contents or try rephrasing your question."

- **What happens when student highlights text spanning multiple paragraphs or sections?**
  - System uses the full highlighted text as context, potentially truncating if it exceeds maximum context length (reasonable default: 2000 characters)

- **What happens when the retrieval service is temporarily unavailable?**
  - System displays user-friendly error: "Unable to retrieve content at this moment. Please try again in a few seconds."

- **What happens when student asks question in non-English language?**
  - System responds in English: "This chatbot answers questions in English based on the Physical-AI & Humanoid Robotics textbook."

- **What happens when student tries to ask questions outside the textbook domain (e.g., weather, news)?**
  - System responds: "I can only answer questions about the Physical-AI & Humanoid Robotics textbook content."

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a floating chat widget visible on all textbook pages that students can expand/collapse
- **FR-002**: System MUST accept free-text questions from students via the chat widget text input
- **FR-003**: System MUST detect when text is highlighted/selected on the page and accept it as contextual input
- **FR-004**: System MUST retrieve 2-5 relevant textbook passages for each student query based on semantic similarity
- **FR-005**: System MUST generate answers synthesizing retrieved passages without fabricating information using OpenAI Agents SDK for intelligent tool orchestration
- **FR-006**: System MUST include source references in the format "Source: Module [Name], Chapter [Name]" for each answer
- **FR-007**: System MUST refuse to answer questions about harmful robotics applications and redirect to Ethics & Safety section
- **FR-008**: System MUST respond "This topic is not covered in the textbook" when no relevant passages are found
- **FR-009**: System MUST prioritize highlighted text as primary context when provided by the student
- **FR-010**: System MUST maintain conversation history within the current browser session only
- **FR-011**: System MUST clear conversation history when the page is refreshed or the widget is closed and reopened
- **FR-012**: System MUST chunk and store textbook content to enable passage retrieval
- **FR-013**: System MUST handle questions in English language only
- **FR-014**: System MUST refuse questions outside the textbook domain (e.g., general knowledge, current events)
- **FR-015**: System MUST provide clear error messages when retrieval or answer generation fails
- **FR-016**: System MUST use OpenAI Agents SDK with function-based tool architecture for RAG retrieval orchestration
- **FR-017**: System MUST support both standard (blocking) and streaming response modes via OpenAI Agents SDK Runner
- **FR-018**: Chat widget MUST support both light and dark theme modes matching the textbook's cyberpunk aesthetic
- **FR-019**: Chat widget MUST use color palette consistent with the custom Docusaurus theme (cyan, purple, pink gradients)
- **FR-020**: Chat widget MUST be accessible from the custom home page with hero section and module cards

### Assumptions

- **Assumption 1**: Textbook content is available in markdown format with consistent structure (modules, chapters, sections)
- **Assumption 2**: Students have internet connectivity to access the chat widget service
- **Assumption 3**: Average query response time target is 2-3 seconds based on standard web application expectations
- **Assumption 4**: Chat widget will be used during active study sessions, not for persistent conversations across days
- **Assumption 5**: Textbook content updates will require manual re-indexing (no automatic content sync)
- **Assumption 6**: No student authentication required since textbook is publicly accessible
- **Assumption 7**: Docusaurus theme uses custom cyberpunk-inspired CSS with light/dark mode support
- **Assumption 8**: Custom home page exists with hero section, module cards, and gradient backgrounds

### Key Entities

- **Question**: Student's text input query with optional highlighted text context and conversation history
- **TextbookPassage**: A semantic chunk of textbook content with associated metadata (module name, chapter name, section heading, file path)
- **Answer**: Generated response combining retrieved passages with source references
- **Conversation**: Temporary session history containing messages and context, stored only in browser session
- **SourceReference**: Citation linking answer content back to specific module and chapter
- **Agent**: OpenAI Agents SDK agent instance configured as "Physical-AI Tutor" with retrieval tool capabilities
- **RetrievalTool**: Function tool decorated with @function_tool that performs embedding → vector search → metadata fetch pipeline

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students receive relevant answers (based on textbook content) within 3 seconds for 95% of queries
- **SC-002**: System correctly identifies "not covered" scenarios and avoids hallucination 100% of the time when no relevant content exists
- **SC-003**: Every answer includes source references (module/chapter names) for 100% of successful responses
- **SC-004**: Students can successfully ask questions about any module or chapter and receive accurate answers 90% of the time
- **SC-005**: Highlighted text explanations reference the highlighted content in 100% of cases when text is selected
- **SC-006**: Chat widget loads and becomes interactive within 1 second of page load
- **SC-007**: System handles at least 100 concurrent student queries without degradation
- **SC-008**: Conversation context is maintained correctly for follow-up questions within 90% of multi-turn conversations

### Qualitative Outcomes

- Students report improved comprehension when using the chatbot for difficult passages
- Students find the chat widget unobtrusive and easily accessible when needed
- Students trust the answers because source references enable verification

## Design & Styling Requirements

### Theme Integration
- **Light Mode**: Uses soft blue/cyan gradients (#0891b2, #2563eb) with clean white backgrounds
  - Hero section: `linear-gradient(135deg, #f0f9ff 0%, #e0f2fe 50%, #bae6fd 100%)`
  - Primary buttons: `linear-gradient(135deg, #0891b2, #2563eb)` with shadow effects
  - Text colors: `#0f172a` (primary), `#475569` (secondary)

- **Dark Mode** (`[data-theme='dark']`): Cyberpunk aesthetic with neon cyan/purple/pink
  - Background: `#0a0e1a` (cyber-dark), `#050810` (cyber-darker)
  - Primary cyan: `#00d9ff` with glow effects
  - Accent purple: `#a855f7`
  - Accent pink: `#ec4899`
  - Gradient buttons and headings with drop-shadow animations

### Chat Widget Styling Requirements
- **Button**: Fixed position (bottom-right), circular 60px, purple gradient background
- **Window**: 400px × 600px, rounded corners (16px), dark mode backdrop-filter blur
- **Messages**: User messages (purple gradient), assistant messages (white/dark background)
- **Highlighted text box**: Blue-tinted background with border, dismissible
- **Responsive**: Full-width on mobile (<768px), maintain bottom positioning
- **Dark mode support**: All components must adapt to `[data-theme='dark']` attribute

### Home Page Integration
- **Hero Section**: Full-height gradient background with animated grid overlay
  - Title: Gradient text with glow animation (cyan → purple → pink)
  - CTA buttons: Primary (gradient), secondary (outlined with hover effects)
  - Scroll indicator with bounce animation

- **Modules Section**: 4 module cards in responsive grid
  - Card hover effects: translateY(-8px), border color change, shadow glow
  - Module numbers: Large gradient text
  - Topics: Bulleted list with custom arrow markers (▹)
  - Links: Cyan color with smooth transitions

## Out of Scope

- **Questions outside textbook domain**: No general knowledge, current events, or unrelated topics
- **Real robot control**: Chatbot does not interface with physical robots or simulation environments
- **Multi-language support**: English only
- **Persistent user profiles**: No accounts, login, or cross-session history
- **Administrative dashboard**: No analytics, usage tracking, or content management interface
- **Voice input/output**: Text-based interaction only
- **Mobile native app**: Web-only interface embedded in Docusaurus site
- **Automatic textbook updates**: Manual re-indexing required when content changes
- **Custom chat styling per user**: Standardized widget appearance (must follow design system)
- **Integration with learning management systems (LMS)**: Standalone feature
- **Custom theme builder**: Fixed cyberpunk theme with light/dark variants only

## Dependencies

- **Textbook content**: Requires access to markdown files with consistent structure and YAML frontmatter
- **Hosting infrastructure**: Requires deployed backend service accessible from textbook website
- **External services**:
  - OpenAI API for embeddings (text-embedding-3-small) and LLM (gpt-4o-mini) via OpenAI Agents SDK
  - Qdrant Cloud for vector storage
  - Neon Postgres for metadata storage
  - Assumed reasonable uptime SLAs for all external services
- **OpenAI Agents SDK**: Requires openai-agents Python package (>=1.0.0) and openai SDK (>=1.33.0)
- **Frontend framework**: Docusaurus 3.x with React 18+ for custom home page and chat widget
- **Styling dependencies**: Custom CSS with cyberpunk theme (`textbook/src/css/custom.css`), CSS modules for ChatWidget

## Constraints

- **Session-only storage**: Conversation history must not persist beyond browser session per constitution principle XIII
- **Content grounding**: All answers must be based on retrieved textbook passages per constitution principle VII
- **Safety compliance**: Must refuse harmful queries per constitution principle IX
- **Simple references**: Must use "Module X, Chapter Y" format per constitution principle VIII
- **Public access**: No authentication barrier since textbook is publicly available
- **Design consistency**: Chat widget styling must match Docusaurus cyberpunk theme color palette and design patterns
- **Theme responsiveness**: All UI components must support seamless light/dark mode switching via `[data-theme]` attribute
- **Performance**: Chat widget animations and theme transitions must not impact textbook page load performance
