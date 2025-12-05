<!--
SYNC IMPACT REPORT
==================
Version Change: 1.0.0 → 1.1.0
Ratification Date: 2025-12-05
Last Amended: 2025-12-06

Amendment Summary (v1.1.0):
- ENHANCED: Educational Excellence principle - replaced vague "Engagement" with measurable "Engagement Standards"
- NEW: Engagement Metrics section - quantifiable interaction requirements (code examples, visual aids, learning checks)
- NEW: Pedagogical Structure section - learning objectives, Bloom's taxonomy, concept mapping
- ENHANCED: Quality Assurance - added Pedagogical Completeness and Pedagogical Verification checklists
- ENHANCED: Success Metrics - added Pedagogical Targets with 6 new quantitative measures
- IMPROVED: Qualitative Measures - replaced subjective criteria with verifiable standards

Detailed Changes:
- Educational Excellence (line 73): "Engagement" → "Engagement Standards" (with reference to Content Standards)
- Content Standards: Added "Engagement Metrics" subsection (lines 133-143)
- Content Standards: Added "Pedagogical Structure" subsection (lines 145-159)
- Quality Assurance: Added "Pedagogical Completeness" checklist (lines 242-249)
- Quality Assurance: Added "Pedagogical Verification" checklist (lines 282-289)
- Success Metrics: Added "Pedagogical Targets" section (lines 363-370)
- Qualitative Measures: Replaced "Pedagogical Value" with testable "Learning Progression" (line 376)

Templates Requiring Updates:
⚠️ Consider reviewing plan-template.md for learning objectives checklist
⚠️ Consider reviewing spec-template.md for Bloom's taxonomy mapping section
✅ tasks-template.md - No changes needed

Follow-up TODOs:
- Consider creating pedagogical review checklist tool
- Document Bloom's taxonomy classification methodology
- Create example concept dependency map template

Rationale for MINOR version (1.1.0):
- Backward-compatible enhancement (doesn't break existing content)
- Adds measurable specifications to existing Educational Excellence principle
- Transforms subjective standards into quantifiable, testable metrics
- No changes to core principles or governance structure
==================
-->

# Teaching Physical AI & Humanoid Robotics Course Constitution

## Project Identity

**Project Name**: Teaching Physical AI & Humanoid Robotics Course
**Platform**: Docusaurus + Spec-Kit Plus
**Deployment**: GitHub Pages
**Primary Tools**: Claude Code, Spec-Kit Plus
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
- **Engagement Standards**: MUST meet quantifiable interaction and learning assessment metrics (see Content Standards)

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

### Engagement Metrics

**Requirements**:

- **Practical Examples**: Minimum 2 code examples per 1,000 words
- **Visual Aids**: Minimum 1 diagram/figure per major concept
- **Learning Checks**: End-of-section questions or exercises (minimum 3 per chapter)
- **Interactive Elements**: Code demonstrations must be executable and testable
- **Real-World Applications**: Each major concept must include practical use case

**Rationale**: Quantifiable engagement metrics ensure content moves beyond passive reading to active learning. Students retain knowledge better through practical application and self-assessment.

### Pedagogical Structure

**Requirements**:

- **Concept Dependency Map**: Document prerequisite relationships between sections in each chapter introduction
- **Learning Objectives**: Explicit, measurable objectives at chapter start (minimum 3 per chapter)
- **Bloom's Taxonomy Balance**:
  - Remembering: 20% (definitions, terminology)
  - Understanding: 30% (explanations, comparisons)
  - Applying: 30% (code examples, exercises)
  - Analyzing: 20% (debugging, optimization, design choices)
- **Progressive Complexity**: Each section builds on previous knowledge with explicit references
- **Assessment Alignment**: Learning checks must map directly to stated learning objectives

**Rationale**: Structured pedagogical design ensures coherent learning progression and enables students to self-assess their mastery. Bloom's taxonomy balance prevents overemphasis on rote memorization while maintaining foundational knowledge.

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

**Pedagogical Completeness**:

- Minimum 2 code examples per 1,000 words
- Minimum 1 diagram/figure per major concept
- Minimum 3 learning checks per chapter
- Learning objectives stated for each chapter (minimum 3)
- Concept dependency map included in chapter introductions
- Bloom's taxonomy balance verified across content

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

**Pedagogical Verification**:

- Learning objectives align with chapter content
- Learning checks map to stated objectives
- Code examples are executable and tested
- Concept dependencies documented and accurate
- Bloom's taxonomy distribution meets 20/30/30/20 target
- Visual aids appropriately support explanations

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

**Pedagogical Targets**:

- **Code Example Density**: Minimum 2 examples per 1,000 words
- **Visual Aid Density**: Minimum 1 diagram/figure per major concept
- **Learning Objectives**: Minimum 3 per chapter, all measurable
- **Learning Checks**: Minimum 3 per chapter, aligned with objectives
- **Bloom's Balance**: 20% remembering, 30% understanding, 30% applying, 20% analyzing (±5% tolerance)
- **Concept Maps**: Complete prerequisite documentation in 100% of chapter introductions

### Qualitative Measures

**Assessment Criteria**:

- **Learning Progression**: Concept dependencies form directed acyclic graph with no circular prerequisites
- **Technical Accuracy**: Verifiable against primary sources
- **Professional Presentation**: Publication-ready formatting
- **Reproducibility**: Independent verification possible with documented environment/dependencies
- **Educational Impact**: Addresses documented gap in existing materials or improves upon existing pedagogy

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

**Version**: 1.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-06
