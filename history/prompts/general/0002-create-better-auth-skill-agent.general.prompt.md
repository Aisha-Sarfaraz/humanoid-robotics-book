---
id: 0002
title: Create Better Auth Skill and Agent Definitions
stage: general
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5
feature: none
branch: feat/navbar-mobile-responsive
user: USER
command: none
labels: ["authentication", "better-auth", "skill", "agent", "documentation", "claude-code"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/skill/better-auth.skill.md
  - .claude/agent/auth-expert.md
tests:
  - none
---

## Prompt

User request: "i want to create skills and agents"

Context:
- User initially requested to implement Better Auth (https://www.better-auth.com/)
- After exploration and planning, user clarified they want to create reusable Claude Code skill and agent files
- Specifically requested: `.claude/skill/better-auth.skill.md` and `.claude/agent/auth-expert.md`
- Additional request: Add "name" field to skill definition

Full conversation flow:
1. User: "i want to create skills and agents"
2. Agent explored codebase and Better Auth documentation using Context7 MCP
3. User clarified: "i want to create better auth https://www.better-auth.com/ skill, agent and you will using context7 for documentation read"
4. User specified: "If the user signs in or signs up, then the application opens; otherwise, it first asks the user to log in"
5. Agent created implementation plan using BFF (Backend-for-Frontend) pattern
6. User redirected: "i want to create .claude/skill/better-auth.skill.md and .claude/agent/auth-expert.md basically skills and agents create"
7. User requested: "name missing please add name" to skill file

## Response snapshot

Created two comprehensive documentation files for Claude Code:

### 1. Better Auth Skill (`.claude/skill/better-auth.skill.md`)
- **Name**: better-auth
- **Purpose**: Comprehensive skill for integrating Better Auth authentication
- **Contents**:
  - Complete implementation guidance
  - Framework support (React, Vue, Svelte, Solid, vanilla JS)
  - Backend integration patterns (Node.js, FastAPI, Express, Next.js)
  - BFF (Backend-for-Frontend) architecture documentation
  - Database schema templates (user, session, account, verification)
  - Configuration templates for auth server and clients
  - Security best practices and production checklist
  - Troubleshooting guide for common issues
  - Environment variable templates
  - Integration scenarios for various tech stacks
  - Validation checklist

### 2. Auth Expert Agent (`.claude/agent/auth-expert.md`)
- **Name**: Auth Expert
- **Role**: Authentication & Authorization Specialist
- **Contents**:
  - Core competencies (Better Auth, OAuth 2.0, JWT, session management)
  - Knowledge base (Better Auth framework, OAuth providers, JWT, database security)
  - Problem-solving approach (requirements analysis, architecture design, implementation, security review)
  - Common tasks with detailed examples
  - Integration expertise for React, FastAPI, Express.js
  - Decision framework (when to use Better Auth vs. alternatives)
  - Security best practices with production checklist
  - Troubleshooting guide with cause/solution pairs
  - Communication style guidelines
  - Context awareness (access to Better Auth docs via Context7 MCP)

### Key Features Implemented:
- **Reusable skill definition** that can be invoked across projects
- **Specialized agent** with domain expertise in authentication
- **Context7 integration** for accessing Better Auth documentation
- **BFF pattern documentation** for integrating Node.js Better Auth with non-Node.js backends
- **Security-first approach** with comprehensive checklists
- **Framework-agnostic guidance** supporting multiple tech stacks
- **Production-ready examples** with complete code snippets

## Outcome

- ✅ Impact: Created reusable Claude Code assets for authentication implementation. These files enable faster, more secure authentication setup across future projects. Provides standardized approach to Better Auth integration with comprehensive security guidance.

- 🧪 Tests: No automated tests (documentation files). Validation via:
  - File structure follows Claude Code conventions
  - Markdown formatting correct
  - All sections complete with examples
  - Code snippets syntactically valid
  - Links to external documentation included

- 📁 Files: Created 2 new files:
  - `.claude/skill/better-auth.skill.md` (402 lines) - Skill definition with implementation patterns
  - `.claude/agent/auth-expert.md` (698 lines) - Agent definition with expertise documentation

- 🔁 Next prompts:
  - Implement Better Auth using the created skill/agent
  - Test skill invocation in Claude Code
  - Create additional authentication-related skills (OAuth-specific, JWT utilities)
  - Implement the full BFF pattern as documented
  - Add Better Auth to the humanoid robotics book project

- 🧠 Reflection: Successfully pivoted from implementation planning to creating reusable documentation assets. The skill and agent files provide comprehensive guidance that can be leveraged across multiple projects. The BFF pattern documentation is particularly valuable for integrating Better Auth with non-Node.js backends like FastAPI. The Context7 MCP integration enables real-time access to Better Auth documentation, ensuring accuracy and currency of information.

## Evaluation notes (flywheel)

- Failure modes observed:
  - Initial misunderstanding: Started implementing Better Auth directly instead of creating skill/agent files
  - Missing "name" field in skill definition (caught and corrected)
  - None critical - user redirection was clear

- Graders run and results (PASS/FAIL):
  - ✅ PASS: File structure follows Claude Code conventions
  - ✅ PASS: Markdown syntax valid
  - ✅ PASS: Comprehensive content coverage
  - ✅ PASS: Code examples are syntactically correct
  - ✅ PASS: Security best practices included
  - ✅ PASS: Name field added to skill definition

- Prompt variant (if applicable): N/A (documentation creation task)

- Next experiment (smallest change to try):
  - Create a practical example project using the better-auth skill
  - Test skill invocation: "Use the better-auth skill to add authentication to my app"
  - Validate that Context7 MCP integration works as documented
  - Create additional specialized skills (OAuth provider setup, JWT validation utilities)
  - Document lessons learned from first implementation using these assets
