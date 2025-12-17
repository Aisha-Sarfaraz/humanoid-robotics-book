<!--
SYNC IMPACT REPORT
==================
Version Change: 0.0.0 → 1.0.0
Ratification Date: 2025-12-05
Last Amended: 2025-12-05

Modified Principles:
- NEW: Academic Integrity (Primary Source Verification, Zero Plagiarism Tolerance, Intellectual Honesty, Peer Review Standards)
- NEW: Educational Excellence (Clarity First, Pedagogical Structure, Practical Application, Engagement)
- NEW: Reproducibility (Citation Traceability, Code Examples, Data Transparency, Version Control)
- NEW: Technical Rigor (Accuracy, Currency, Depth, Scope Management)

Added Sections:
- Content Standards (Writing Quality, Citation Requirements)
- Technical Specifications (Document Structure, Format Requirements, Docusaurus Configuration)
- Development Workflow (Phase 1-4: Planning, Writing, Review & Refinement, Deployment)
- Quality Assurance Checklist (Pre-Submission Requirements, Content Verification)
- Prohibited Practices (Absolutely Forbidden, Discouraged Practices)
- Source Evaluation Criteria (Peer-Reviewed Articles, Conference Proceedings, Technical Reports)
- Success Metrics (Quantitative Measures, Qualitative Measures)
- Maintenance & Updates (Post-Launch Responsibilities, Version Control Standards)
- Legal & Ethical Compliance (Copyright & Licensing, Academic Ethics)
- Tool-Specific Guidelines (Claude Code Usage, Spec-Kit Plus Integration, GitHub Workflow)

Templates Requiring Updates:
✅ plan-template.md - Reviewed (no changes needed - Constitution Check section already flexible)
✅ spec-template.md - Reviewed (no changes needed - compatible with academic content structure)
✅ tasks-template.md - Reviewed (no changes needed - task structure supports documentation workflows)
✅ Command files - None requiring updates (commands are agent-agnostic)

Follow-up TODOs:
- None - all placeholders filled with concrete values

Rationale for MAJOR version (1.0.0):
- Initial constitution ratification for new project
- Establishes foundational principles and governance
- Defines complete development workflow and standards
==================
-->

# Teaching Physical AI & Humanoid Robotics Course Constitution

## Project Identity

**Project Name**: Teaching Physical AI & Humanoid Robotics Course with Integrated RAG Chatbot
**Platform**: Docusaurus + FastAPI + Spec-Kit Plus
**Deployment**: GitHub Pages + Backend API
**Primary Tools**: Claude Code, Spec-Kit Plus, FastAPI, OpenAI APIs, Qdrant, Neon Postgres
**Target Audience**: Computer science students and academics
**Knowledge Level**: Advanced undergraduate/graduate level

## Core Principles

### I. Academic Integrity

**Non-Negotiable Requirements**:

- **Primary Source Verification**: Every factual claim MUST be traceable to authoritative, verifiable sources
- **Zero Plagiarism Tolerance**: All content MUST pass plagiarism detection with 0% similarity
- **Intellectual Honesty**: Proper attribution MUST be provided for all ideas, concepts, and data
- **Peer Review Standards**: MUST prioritize peer-reviewed academic publications

**Rationale**: Academic integrity forms the foundation of educational credibility. Without verifiable sources and original content, the course material loses its value as a teaching resource and violates academic ethics standards.

### II. Educational Excellence

**Non-Negotiable Requirements**:

- **Clarity First**: Content MUST be accessible to readers with computer science backgrounds
- **Pedagogical Structure**: MUST follow logical progression from foundational to advanced concepts
- **Practical Application**: MUST balance theory with hands-on, reproducible examples
- **Engagement**: MUST maintain academic rigor while keeping content compelling

**Rationale**: Educational content that fails to engage or educate effectively wastes student time and undermines learning objectives. Clear structure and practical examples ensure knowledge transfer.

### III. Reproducibility

**Non-Negotiable Requirements**:

- **Citation Traceability**: Every claim MUST include proper citation enabling readers to verify sources
- **Code Examples**: All technical implementations MUST be testable and reproducible
- **Data Transparency**: Datasets, algorithms, and methodologies MUST be clearly documented
- **Version Control**: All changes MUST be tracked through Git with meaningful commit messages

**Rationale**: Reproducibility is fundamental to scientific and educational integrity. Students and instructors must be able to verify claims and reproduce results independently.

### IV. Technical Rigor

**Non-Negotiable Requirements**:

- **Accuracy**: Technical details MUST be verified against multiple authoritative sources
- **Currency**: MUST focus on current state-of-the-art while acknowledging historical context
- **Depth**: MUST provide sufficient technical detail for implementation and understanding
- **Scope Management**: MUST provide comprehensive coverage within defined boundaries

**Rationale**: Technical education demands precision. Inaccurate or outdated information actively harms student learning and career preparation.

### V. RAG Chatbot Enhancement

**Non-Negotiable Requirements**:

- **Content Fidelity**: Chatbot responses MUST be grounded in actual book content with minimal hallucination
- **Context Awareness**: System MUST support both full-book queries and text selection-based queries
- **Response Quality**: Answers MUST maintain the same academic rigor as the original content
- **Performance**: System MUST respond within 3 seconds under normal load conditions
- **Integration**: Chatbot MUST integrate seamlessly with existing Docusaurus structure

**Rationale**: The RAG chatbot extends the educational value of the course material by providing interactive access to content while maintaining academic integrity and user experience standards.

## Content Standards

### Writing Quality

**Requirements**:

- **Readability**: Flesch-Kincaid Grade Level 10-12
- **Tone**: Professional academic writing, avoiding colloquialisms
- **Structure**: Clear hierarchy with descriptive headings and subheadings
- **Consistency**: Uniform terminology, notation, and formatting throughout

### Citation Requirements

**Format**: APA Style (7th Edition)
**Minimum Sources**: 15 scholarly sources
**Source Distribution**: Minimum 50% peer-reviewed journal articles

**Source Types** (in priority order):

1. Peer-reviewed journal articles (primary)
2. Conference proceedings (secondary)
3. Technical reports from reputable institutions
4. Official documentation (for frameworks/tools)
5. Books from academic publishers (supplementary)

**Citation Standards**:

- **In-text Citations**: Author-date format: (Smith, 2023)
- **Direct Quotes**: Page numbers required: (Smith, 2023, p. 45)
- **Multiple Authors**:
  - 1-2 authors: (Smith & Jones, 2023)
  - 3+ authors: (Smith et al., 2023)
- **Reference List**: Complete bibliographic information at end of each chapter

## Technical Specifications

### Document Structure

**Requirements**:

- **Word Count**: 5,000-7,000 words (excluding code blocks and references)
- **Chapter Organization**:
  - Introduction (10-15% of content)
  - Main chapters (70-75% of content)
  - Conclusion (10-15% of content)
  - References (separate section)
- **Subsections**: Maximum 3 levels of hierarchy
- **Code Blocks**: Properly formatted with syntax highlighting

### Format Requirements

**Requirements**:

- **Primary Format**: Markdown (for Docusaurus)
- **Export Format**: PDF with embedded citations
- **Images**: High-resolution (minimum 300 DPI), properly attributed
- **Diagrams**: Vector format preferred (SVG), created or licensed appropriately
- **Tables**: Clear labels, captions, and sources

### Docusaurus Configuration

**Requirements**:

- **Sidebar Navigation**: Logical chapter/section hierarchy
- **Search Functionality**: Enabled with appropriate indexing
- **Responsive Design**: Mobile-friendly layout
- **Accessibility**: WCAG 2.1 AA compliance
- **Performance**: Page load time under 3 seconds

## Development Workflow

### Phase 1: Planning

**Tasks**:

- **Outline Development**: Create detailed table of contents, define learning objectives per chapter, map source requirements to sections
- **Source Collection**: Compile bibliography of 20+ potential sources, verify accessibility of all sources, organize sources by topic/chapter
- **Spec-Kit Plus Setup**: Initialize project structure, configure Docusaurus settings, set up GitHub repository

### Phase 2: Writing

**Tasks**:

- **Chapter Development**: Write in order (intro to conclusion), include citations during writing (not after), draft code examples alongside text
- **Daily Quality Checks**: Run plagiarism detection on new content, verify all citations have complete references, test all code examples
- **Version Control**: Commit completed sections daily, use descriptive commit messages, create branches for major chapters

### Phase 3: Review & Refinement

**Tasks**:

- **Technical Review**: Verify all technical claims against sources, test reproducibility of all examples, check mathematical notation consistency
- **Editorial Review**: Readability assessment (Flesch-Kincaid), grammar and style consistency, citation format verification
- **Quality Assurance**: Final plagiarism check (must be 0%), complete fact-checking review, cross-reference verification

### Phase 4: Deployment

**Tasks**:

- **GitHub Pages Setup**: Configure deployment workflow, test live site functionality, verify all links and resources
- **PDF Generation**: Export with embedded citations, verify formatting in PDF, test hyperlinks
- **Final Documentation**: Update README with usage instructions, document build process, include license information

## Quality Assurance

### Pre-Submission Requirements

**Content Completeness**:

- Total word count: 5,000-7,000 words
- Minimum 15 scholarly sources cited
- 50%+ peer-reviewed sources
- All citations in APA format
- Complete reference list

**Quality Gates**:

- Plagiarism check result: 0%
- Flesch-Kincaid: Grade 10-12
- All code examples tested and functional
- All images properly attributed
- Fact-checking review passed

**Technical Requirements**:

- Docusaurus builds without errors
- GitHub Pages deployment successful
- PDF export with intact citations
- Accessibility standards met (WCAG 2.1 AA)

### Content Verification

**Citation Verification**:

- Every factual claim has citation
- All URLs accessible and working
- Technical terminology consistent
- Mathematical notation standardized

**Structural Verification**:

- Code syntax highlighting correct
- Section numbering sequential
- Cross-references accurate
- Index/glossary complete (if included)

## Prohibited Practices

### Absolutely Forbidden

**Zero-Tolerance Violations**:

- **Plagiarism**: Any uncited copying of text, even with minor modifications
- **Fabrication**: Creating citations for non-existent sources
- **Self-plagiarism**: Reusing previous work without citation
- **Predatory Sources**: Citations from predatory journals or unverified websites
- **Outdated Information**: Relying solely on sources >10 years old without justification

### Discouraged Practices

**Avoid Unless Justified**:

- Over-reliance on secondary sources
- Excessive direct quotations (max 10% of content)
- Wikipedia or non-peer-reviewed web sources as primary references
- Incomplete citations missing page numbers or DOIs
- Inconsistent terminology across chapters

## Source Evaluation Criteria

### Peer-Reviewed Articles (Preferred)

**Criteria**:

- Published in indexed journals (IEEE, ACM, Springer, etc.)
- Impact factor consideration
- Recent publication (within 5 years preferred)
- Relevance to physical AI/robotics

### Conference Proceedings (Acceptable)

**Criteria**:

- Major conferences (NeurIPS, ICRA, IROS, CVPR, etc.)
- Peer-reviewed proceedings
- Recent presentations

### Technical Reports (Supplementary)

**Acceptable Sources**:

- From reputable institutions (MIT, Stanford, CMU, etc.)
- From industry leaders (Google AI, OpenAI, Boston Dynamics, etc.)
- Official documentation for frameworks/tools

### Unacceptable Sources

**Prohibited**:

- Non-peer-reviewed blog posts
- Wikipedia (may use for initial research, not citation)
- Social media content
- Press releases (unless analyzing industry claims)
- Undergraduate student papers

## Success Metrics

### Quantitative Measures

**Measurable Targets**:

- **Citation Density**: Minimum 1 citation per 300 words
- **Source Diversity**: Citations from 15+ unique sources
- **Code Functionality**: 100% of examples run without errors
- **Build Success**: Zero errors in Docusaurus build
- **Load Performance**: < 3 seconds page load time
- **Accessibility Score**: 95%+ on Lighthouse audit

### Qualitative Measures

**Assessment Criteria**:

- **Pedagogical Value**: Clear learning progression
- **Technical Accuracy**: Verifiable against primary sources
- **Professional Presentation**: Publication-ready formatting
- **Reproducibility**: Independent verification possible
- **Contribution**: Fills gap in existing educational materials

## Maintenance & Updates

### Post-Launch Responsibilities

**Ongoing Tasks**:

- **Issue Tracking**: Monitor and respond to GitHub issues
- **Content Updates**: Review and update outdated information annually
- **Link Maintenance**: Verify all external links quarterly
- **Community Engagement**: Respond to feedback and suggestions
- **Version History**: Maintain changelog for all significant updates

### Version Control Standards

**Requirements**:

- **Semantic Versioning**: MAJOR.MINOR.PATCH
- **Release Notes**: Document all changes per version
- **Archival**: Maintain previous versions for reference

## Legal & Ethical Compliance

### Copyright & Licensing

**Requirements**:

- **Content License**: Specify (e.g., Creative Commons BY-NC-SA 4.0)
- **Code License**: Specify (e.g., MIT License)
- **Image Rights**: Verify permission for all non-original images
- **Attribution**: Complete credit for all contributors

### Academic Ethics

**Requirements**:

- **Conflict of Interest**: Disclose any relevant affiliations
- **Human Subjects**: N/A for this project type
- **Data Privacy**: Ensure no personal data in examples
- **Responsible AI**: Discuss ethical implications of technologies

## Tool-Specific Guidelines

### Claude Code Usage

**Requirements**:

- Use for content generation, never for citation fabrication
- Always verify AI-generated claims against sources
- Use for code example generation and testing
- Document AI assistance in development process

### Spec-Kit Plus Integration

**Requirements**:

- Follow framework documentation precisely
- Maintain compatibility with Docusaurus versions
- Test all custom components thoroughly
- Document any framework modifications

### GitHub Workflow

**Requirements**:

- Meaningful commit messages describing changes
- Branch protection for main branch
- Pull request reviews for major changes
- Automated testing with GitHub Actions

## Appendices

### A. APA Citation Quick Reference

**Journal Article**: Author, A. A. (Year). Title of article. *Journal Name*, *Volume*(Issue), pages. DOI

**Book**: Author, A. A. (Year). *Title of book*. Publisher.

**Conference Paper**: Author, A. A. (Year). Title of paper. In *Proceedings of Conference* (pp. pages). Publisher.

### B. Recommended Journals

- IEEE Transactions on Robotics
- International Journal of Robotics Research
- Robotics and Autonomous Systems
- Journal of Machine Learning Research
- Neural Computation

### C. Useful Resources

- **Google Scholar** for academic search
- **Connected Papers** for citation mapping
- **Zotero/Mendeley** for reference management
- **Grammarly Academic** for writing assistance
- **Turnitin/Copyscape** for plagiarism detection

## Governance

### Amendment Process

This constitution may be amended when:

- New pedagogical standards emerge
- Platform requirements change (Docusaurus updates)
- Academic citation standards evolve
- Project scope significantly expands

**Amendment Procedure**:

1. Propose changes in writing with justification
2. Review against core principles
3. Document amendment in version control
4. Update all affected workflows and templates

### Compliance

**Requirements**:

- All content development MUST verify compliance with this constitution
- Pull requests MUST include constitution compliance checklist
- Complexity or deviations MUST be justified and documented
- Regular audits MUST be conducted to ensure ongoing compliance

### Constitution Supersedes All Other Practices

This constitution represents the highest authority for project governance. When conflicts arise between this document and other guidance, the constitution takes precedence. All project artifacts, workflows, and decisions MUST align with these principles.

---

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
