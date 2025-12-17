# Teaching Physical AI & Humanoid Robotics

**A Resource-Aware Pedagogical Framework for Computer Science Education**

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

## Overview

This academic book provides a comprehensive framework for teaching Physical AI and Humanoid Robotics in resource-constrained university environments. Using a simulation-first approach, the curriculum enables institutions to offer complete Physical AI courses regardless of budget limitations.

**Key Features**:
- 📚 Complete 13-week curriculum with detailed module breakdowns
- 💰 Three budget tiers ($5K, $15K, $50K+) with hardware recommendations
- 🎓 Evidence-based pedagogical strategies from peer-reviewed research
- 🔬 5+ detailed lab exercises with assessment rubrics
- 🛡️ Comprehensive safety protocols for robotics labs
- 🌐 Remote learning alternatives for equitable access
- 🤖 **NEW**: Integrated RAG Chatbot for interactive learning

## Target Audience

- **University Instructors**: CS faculty launching first Physical AI course
- **Graduate Students**: Exploring research directions in embodied intelligence
- **Industry Trainers**: Designing intensive robotics upskilling programs

## Quick Start

### Prerequisites

- Node.js 18+ and npm
- Git
- Basic understanding of computer science education

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/humanoid-robotics-book.git
cd humanoid-robotics-book

# Install dependencies
npm install

# Start local development server
npm start
```

The site will open at `http://localhost:3000`.

### Build for Production

```bash
# Create production build
npm run build

# Test production build locally
npm run serve
```

### Deploy to GitHub Pages

```bash
# Deploy to GitHub Pages
npm run deploy
```

## Book Structure

- **Chapter 1**: Introduction to Physical AI in Education (750-1000 words)
- **Chapter 2**: Applications of Physical AI (1000-1200 words)
- **Chapter 3**: Pedagogical Foundations (800-1000 words)
- **Chapter 4**: Curriculum Design Framework (1200-1500 words)
- **Chapter 5**: Implementation Guide (1500-1800 words)
- **Chapter 6**: Advanced Topics and Research Frontiers (500-700 words)
- **Chapter 7**: Conclusion and Future Directions (300-500 words)

**Total**: 5,000-7,000 words | 15+ peer-reviewed citations

## Curriculum Framework

### 13-Week Course Structure

**Weeks 1-2**: Foundations (Python, Linux, ROS 2 basics)
**Weeks 3-5**: Module 1 - ROS 2 Fundamentals (Nodes, Topics, URDF)
**Weeks 6-7**: Module 2 - Physics Simulation (Gazebo, Unity)
**Weeks 8-10**: Module 3 - NVIDIA Isaac Platform (Isaac Sim, Visual SLAM, Nav2)
**Weeks 11-12**: Module 4 - Vision-Language-Action Models (VLA integration)
**Week 13**: Final Project Presentations

### Hardware Tiers

| Tier | Budget | Use Case | Capabilities |
|------|--------|----------|-------------|
| **Minimal** | $5,000 | Simulation-only | Modules 1-2, cloud for Module 3 |
| **Recommended** | $15,000 | Full curriculum + edge devices | All modules, optional quadruped robot |
| **Premium** | $50,000+ | Research-grade | Humanoid robot, advanced sensors |

## RAG Chatbot Integration

This book now features an integrated Retrieval-Augmented Generation (RAG) chatbot that allows students and instructors to ask questions about the book's content and receive contextual responses.

### Features

- **Interactive Q&A**: Ask questions about any chapter or concept in the book
- **Text Selection Context**: Select text on any page and ask specific questions about it
- **Source Citations**: Responses include citations to relevant book sections
- **Conversation History**: Maintains context across multiple questions
- **Real-time Responses**: Fast, accurate answers powered by AI

### Backend Setup

To run the RAG chatbot backend:

1. Navigate to the backend directory:
```bash
cd backend
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables by creating a `.env` file:
```env
DATABASE_URL=your_neon_postgres_connection_string
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
```

4. Run the backend server:
```bash
python run_backend.py
```

The backend will start on `http://localhost:8000`.

### Chatbot Usage

- Click the 🤖 button in the bottom-right corner to open the chat interface
- Type your question about the book content
- Select text on any page to ask specific questions about that content
- View source citations in the response for reference

## Development

### Project Structure

```
humanoid-robotics-book/
├── backend/                 # FastAPI backend for RAG chatbot
│   ├── app/                 # Application code
│   │   ├── models/          # Database models
│   │   ├── schemas/         # Pydantic schemas
│   │   ├── services/        # Business logic
│   │   ├── utils/           # Utility functions
│   │   └── routers/         # API routes
│   └── requirements.txt     # Python dependencies
├── docs/                    # Book chapters (Markdown)
│   ├── chapter-01/
│   ├── chapter-02/
│   └── ...
├── research/                # Research notes and bibliography
│   └── research-notes/
├── quality/                 # Quality assurance logs
│   ├── daily-logs/
│   ├── plagiarism-reports/
│   └── readability-reports/
├── specs/                   # Planning documents
│   └── 001-teaching-physical-ai-robotics-book/
├── .specify/                # Spec-Kit Plus templates
├── src/                     # Frontend components
│   └── components/          # React components (including ChatInterface)
├── docusaurus.config.js     # Docusaurus configuration
├── sidebars.js              # Sidebar navigation
└── package.json             # Dependencies
```

### Quality Standards

This book adheres to rigorous academic standards:

- ✅ **0% Plagiarism** (verified via Turnitin/Copyscape)
- ✅ **15+ Peer-Reviewed Citations** (50%+ from IEEE, ACM, Springer journals)
- ✅ **Flesch-Kincaid Grade 10-12** readability
- ✅ **WCAG 2.1 AA** accessibility compliance
- ✅ **< 3 second** page load time

### Contributing

This is an academic project developed following the constitution in `.specify/memory/constitution.md`. All changes must maintain academic integrity standards.

## License

**Content**: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)

You are free to:
- **Share**: Copy and redistribute the material
- **Adapt**: Remix, transform, and build upon the material

Under the following terms:
- **Attribution**: Provide appropriate credit
- **NonCommercial**: Not for commercial purposes
- **ShareAlike**: Distribute under the same license

## Citation

If you use this framework in your teaching or research, please cite:

```
Teaching Physical AI & Humanoid Robotics: A Resource-Aware Pedagogical Framework
Version 1.0, December 2025
Available at: https://github.com/your-username/humanoid-robotics-book
License: CC BY-NC-SA 4.0
```

## Contact & Support

- **Issues**: [GitHub Issues](https://github.com/your-username/humanoid-robotics-book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/humanoid-robotics-book/discussions)

## Acknowledgments

This work was developed with AI assistance from Claude (Anthropic) following rigorous academic standards. All content has been independently verified against primary sources. Development followed Spec-Driven Development methodology using Spec-Kit Plus.

---

**Built with [Docusaurus](https://docusaurus.io/)**
