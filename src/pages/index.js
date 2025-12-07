import React, { useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

// Scroll Progress
function ScrollProgress() {
  const [scrollProgress, setScrollProgress] = useState(0);

  useEffect(() => {
    const updateScrollProgress = () => {
      const scrollHeight = document.documentElement.scrollHeight - window.innerHeight;
      const scrolled = (window.scrollY / scrollHeight) * 100;
      setScrollProgress(scrolled);
    };

    window.addEventListener('scroll', updateScrollProgress);
    return () => window.removeEventListener('scroll', updateScrollProgress);
  }, []);

  return <div className="scroll-progress" style={{ width: `${scrollProgress}%` }} />;
}

// Bold Hero Section
function HeroSection() {
  const handleScroll = (e, targetId) => {
    e.preventDefault();
    const element = document.getElementById(targetId);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  };

  return (
    <section className="hero">
      <div className="hero-content">
        <div className="hero-text">
          <h1>
            Teaching <span className="highlight">Physical AI</span> & Humanoid Robotics
          </h1>
          <p className="subtitle">
            A Resource-Aware Pedagogical Framework
          </p>
          <p className="description">
            Bridge the gap between digital intelligence and physical embodiment.
            Master ROS 2, simulation frameworks, and NVIDIA Isaac in academic environments.
          </p>

          <div className="cta-buttons">
            <Link to="docs/intro" className="btn btn-primary">
              Start Learning →
            </Link>
            <a
              href="#features"
              className="btn btn-secondary"
              onClick={(e) => handleScroll(e, 'features')}
            >
              Explore Features
            </a>
          </div>

          <div className="stats">
            <div className="stat-card">
              <div className="stat-number">4</div>
              <div className="stat-label">Modules</div>
            </div>
            <div className="stat-card">
              <div className="stat-number">12</div>
              <div className="stat-label">Weeks</div>
            </div>
            <div className="stat-card">
              <div className="stat-number">100%</div>
              <div className="stat-label">Hands-On</div>
            </div>
          </div>
        </div>

        <div className="hero-visual">
          <div className="visual-container">
            <div className="visual-icon">🤖</div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Features Section
function FeaturesSection() {
  const features = [
    {
      icon: '🎯',
      title: 'Simulation-First',
      description: 'Start with Gazebo and Isaac Sim. Learn without expensive hardware, then transition to physical robots seamlessly.'
    },
    {
      icon: '💡',
      title: 'Budget-Aware',
      description: 'Three-tier approach from $5K simulation-only to $50K+ full lab. Work within your constraints.'
    },
    {
      icon: '🧠',
      title: 'AI-Powered',
      description: 'Integrate Vision-Language-Action models. Voice commands, cognitive planning, and natural language control.'
    },
    {
      icon: '📚',
      title: 'Pedagogically Sound',
      description: 'Evidence-based teaching strategies. Complete with rubrics, exercises, and safety protocols.'
    },
    {
      icon: '⚡',
      title: 'Industry Tools',
      description: 'Master ROS 2, NVIDIA Isaac, Unity, Docker. Skills for robotics careers and research.'
    },
    {
      icon: '🌐',
      title: 'Remote-Ready',
      description: 'Cloud-based alternatives and async labs. Accessible anywhere, democratizing education.'
    }
  ];

  return (
    <section className="section fade-in" id="features">
      <div className="section-header">
        <span className="section-tag">BENEFITS</span>
        <h2>Why This Course?</h2>
        <p>
          Master embodied intelligence through practical, hands-on learning
        </p>
      </div>

      <div className="feature-grid">
        {features.map((feature, idx) => (
          <div className="feature-card" key={idx}>
            <div className="feature-icon">{feature.icon}</div>
            <h3>{feature.title}</h3>
            <p>{feature.description}</p>
          </div>
        ))}
      </div>
    </section>
  );
}

// Modules Section
function ModulesSection() {
  const modules = [
    {
      number: '01',
      title: 'The Robotic Nervous System (ROS 2)',
      description: 'Master middleware for robot control. Nodes, topics, services, URDF modeling. Bridge Python AI to ROS controllers.',
      link: 'docs/chapter-04/module-ros2'
    },
    {
      number: '02',
      title: 'The Digital Twin (Gazebo & Unity)',
      description: 'Physics simulation and environments. Gravity, collisions, sensors (LiDAR, cameras, IMUs), high-fidelity rendering.',
      link: 'docs/chapter-04/module-simulation'
    },
    {
      number: '03',
      title: 'The AI-Robot Brain (NVIDIA Isaac™)',
      description: 'Advanced perception and training. Photorealistic simulation, synthetic data, VSLAM, bipedal path planning.',
      link: 'docs/chapter-04/module-isaac'
    },
    {
      number: '04',
      title: 'Vision-Language-Action (VLA)',
      description: 'LLMs meet robotics. Voice-to-action with Whisper, cognitive planning with GPT, autonomous task execution.',
      link: 'docs/chapter-04/module-vla'
    }
  ];

  return (
    <section className="section fade-in" id="modules">
      <div className="section-header">
        <span className="section-tag">CURRICULUM</span>
        <h2>Course Modules</h2>
        <p>
          Four comprehensive modules covering the complete robotics stack
        </p>
      </div>

      <div className="module-list">
        {modules.map((module, idx) => (
          <Link
            to={module.link}
            key={idx}
            className="module-item"
          >
            <div className="module-number">{module.number}</div>
            <div className="module-content">
              <h3>{module.title}</h3>
              <p>{module.description}</p>
            </div>
            <div className="module-arrow">→</div>
          </Link>
        ))}
      </div>
    </section>
  );
}

// About Section
function AboutSection() {
  const highlights = [
    {
      icon: '📖',
      title: 'Comprehensive',
      description: 'From basics to advanced. Complete coverage of ROS 2, simulation, Isaac, and VLA with real examples.'
    },
    {
      icon: '🎓',
      title: 'Academic',
      description: 'Built for universities. Clear objectives, rigorous assessment, comprehensive safety guidelines.'
    },
    {
      icon: '🔬',
      title: 'Research-Backed',
      description: 'Learning theory and evidence-based strategies. Pedagogical effectiveness guaranteed.'
    }
  ];

  return (
    <section className="section fade-in" id="about">
      <div className="section-header">
        <span className="section-tag">ABOUT</span>
        <h2>Modern Robotics Education</h2>
        <p>
          Academic rigor meets practical innovation
        </p>
      </div>

      <div className="feature-grid">
        {highlights.map((highlight, idx) => (
          <div className="feature-card" key={idx}>
            <div className="feature-icon">{highlight.icon}</div>
            <h3>{highlight.title}</h3>
            <p>{highlight.description}</p>
          </div>
        ))}
      </div>
    </section>
  );
}

// Bold CTA
function CTASection() {
  return (
    <section className="cta-section">
      <div className="cta-content">
        <h2>Ready to Transform Your Program?</h2>
        <p>
          Join leading institutions in adopting a research-backed approach
          to teaching Physical AI and humanoid robotics.
        </p>
        <div className="cta-buttons" style={{ justifyContent: 'center' }}>
          <Link to="docs/intro" className="btn btn-primary">
            Start Reading
          </Link>
          <Link
            to="https://github.com/Aisha-Sarfaraz/humanoid-robotics-book"
            className="btn btn-secondary"
          >
            View GitHub
          </Link>
        </div>
      </div>
    </section>
  );
}

// Scroll Animations
function ScrollAnimations() {
  useEffect(() => {
    const observerOptions = {
      threshold: 0.1,
      rootMargin: '0px 0px -50px 0px'
    };

    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('visible');
        }
      });
    }, observerOptions);

    document.querySelectorAll('.fade-in').forEach(el => {
      observer.observe(el);
    });

    return () => {
      observer.disconnect();
    };
  }, []);

  return null;
}

// Main
export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Teaching Physical AI & Humanoid Robotics: A Resource-Aware Pedagogical Framework">
      <ScrollProgress />
      <ScrollAnimations />
      <main>
        <HeroSection />
        <FeaturesSection />
        <ModulesSection />
        <AboutSection />
        <CTASection />
      </main>
    </Layout>
  );
}
