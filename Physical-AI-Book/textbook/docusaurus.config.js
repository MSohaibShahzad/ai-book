// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical-AI & Humanoid Robotics',
  tagline: 'A University Textbook on Modern Humanoid Robotics',
  favicon: 'img/favicon.ico',

  customFields: {
      apiUrl: process.env.API_URL || 'https://ai-book-ki61.vercel.app/v1',
      authApiUrl: process.env.AUTH_API_URL || 'https://ai-book-ki61.vercel.app',
    },
  // Set the production url of your site here
  url: 'https://ai-book-green.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'msohaibshahzad', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
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
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/MSohaibShahzad/ai-book/Physical-AI-Book/tree/main/textbook',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'static/logo.png',
      navbar: {
        title: '',
        logo: {
          alt: 'Physical-AI Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'doc',
            docId: 'preface/index',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/MSohaibShahzad/ai-book',
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'custom-userMenu',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Modules',
            items: [
              {
                label: 'Preface',
                to: '/docs/preface',
              },
              {
                label: 'Module 1: ROS 2 Foundations',
                to: '/docs/foundations-ros2',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/digital-twin',
              },
              {
                label: 'Module 3: GPU Perception',
                to: '/docs/ai-robot-brain',
              },
              {
                label: 'Module 4: Vision-Language-Action',
                to: '/docs/vision-language-action',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Glossary',
                to: '/docs/appendices/glossary',
              },
              {
                label: 'References',
                to: '/docs/appendices/references',
              },
            ],
          },
          {
            title: 'Connect',
            items: [
              {
                label: '⭐ GitHub Repository',
                href: 'https://github.com/MSohaibShahzad/ai-book',
              },
              {
                label: '👤 GitHub Profile',
                href: 'https://github.com/MSohaibShahzad',
              },
              {
                label: '💼 LinkedIn',
                href: 'https://linkedin.com/in/msohaibshahzad',
              },
              {
                label: '🐦 Twitter',
                href: 'https://twitter.com/msohaibshahzad',
              },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} Physical-AI & Humanoid Robotics Textbook. Licensed under CC BY-NC-SA 4.0.<br/>Built with ❤️ by <a href="https://github.com/MSohaibShahzad" target="_blank" rel="noopener noreferrer" style="color: var(--cyber-cyan);">M. Sohaib Shahzad</a>`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'yaml', 'bash'],
      },
    }),
};

export default config;
