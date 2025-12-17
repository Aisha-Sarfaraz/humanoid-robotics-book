// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Teaching Physical AI & Humanoid Robotics',
  tagline: 'A Resource-Aware Pedagogical Framework',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://aisha-sarfaraz.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/humanoid-robotics-book/',

  // GitHub pages deployment config.
  organizationName: 'Aisha-Sarfaraz', // Usually your GitHub org/user name.
  projectName: 'humanoid-robotics-book', // Usually your repo name.

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',

  // Custom fields for environment variables
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'http://localhost:8000',
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: 'docs',
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          editUrl: 'https://github.com/Aisha-Sarfaraz/humanoid-robotics-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Teaching Physical AI',
        logo: {
          alt: 'Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/',
            label: 'Home',
            position: 'left',
          },
          {
            to: '/docs/chapter-04/module-ros2',
            label: 'Learning Modules',
            position: 'left',
          },
          {
            to: '/docs/category/chapters',
            label: 'All Books',
            position: 'left',
          },
          {
            href: 'https://github.com/Aisha-Sarfaraz/humanoid-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Introduction',
                to: 'docs/intro',
              },
              {
                label: 'Chapter 1: Introduction to Physical AI',
                to: 'docs/chapter-01/introduction',
              },
              {
                label: 'Chapter 4: Complete Modules',
                to: 'docs/chapter-04/module-ros2',
              },
              {
                label: 'All Chapters',
                to: 'docs/category/chapters',
              },
            ],
          },
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: ROS 2',
                to: 'docs/chapter-04/module-ros2',
              },
              {
                label: 'Module 2: Simulation',
                to: 'docs/chapter-04/module-simulation',
              },
              {
                label: 'Module 3: NVIDIA Isaac',
                to: 'docs/chapter-04/module-isaac',
              },
              {
                label: 'Module 4: VLA Models',
                to: 'docs/chapter-04/module-vla',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Aisha-Sarfaraz/humanoid-robotics-book',
              },
              {
                label: 'Appendices',
                to: 'docs/appendices',
              },
              {
                label: 'References',
                to: 'docs/references',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Teaching Physical AI & Humanoid Robotics. Built with Docusaurus. Licensed under CC BY-NC-SA 4.0.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;
