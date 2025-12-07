/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      items: [
        'chapter-01/introduction',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: Applications of Physical AI',
      items: [
        'chapter-02/current-state',
        'chapter-02/healthcare',
        'chapter-02/manufacturing',
        'chapter-02/service-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Pedagogical Foundations',
      items: [
        'chapter-03/learning-theory',
        'chapter-03/teaching-strategies',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: Curriculum Design Framework',
      items: [
        'chapter-04/course-structure',
        'chapter-04/module-ros2',
        'chapter-04/module-simulation',
        'chapter-04/module-isaac',
        'chapter-04/module-vla',
        'chapter-04/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Implementation Guide',
      items: [
        'chapter-05/infrastructure',
        'chapter-05/software',
        'chapter-05/safety',
        'chapter-05/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6: Advanced Topics',
      items: [
        'chapter-06/research',
        'chapter-06/ethics',
        'chapter-06/careers',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 7: Conclusion',
      items: [
        'chapter-07/conclusion',
      ],
    },
    {
      type: 'doc',
      id: 'appendices',
      label: 'Appendices',
    },
    {
      type: 'doc',
      id: 'references',
      label: 'References',
    },
  ],
};

module.exports = sidebars;
