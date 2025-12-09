import React from 'react';
import Link from '@docusaurus/Link';

const BookNavigation = () => {
  return (
    <div className="book-navigation">
      <h3>📚 Book Navigation</h3>
      <ul>
        <li><Link to="/docs/intro">📖 Introduction</Link></li>
        <li><Link to="/docs/chapter-01/introduction">📖 Chapter 1: Introduction to Physical AI</Link></li>
        <li><Link to="/docs/chapter-02/current-state">📖 Chapter 2: Applications</Link></li>
        <li><Link to="/docs/chapter-03/learning-theory">📖 Chapter 3: Pedagogical Foundations</Link></li>
        <li><Link to="/docs/chapter-04/module-ros2">📖 Chapter 4: Curriculum Design (Modules)</Link></li>
        <li><Link to="/docs/chapter-05/infrastructure">📖 Chapter 5: Implementation Guide</Link></li>
        <li><Link to="/docs/chapter-06/research">📖 Chapter 6: Advanced Topics</Link></li>
        <li><Link to="/docs/chapter-07/conclusion">📖 Chapter 7: Conclusion</Link></li>
        <li><Link to="/docs/category/chapters">📖 All Chapters</Link></li>
        <li><Link to="/docs/references">📚 References</Link></li>
      </ul>
    </div>
  );
};

export default BookNavigation;