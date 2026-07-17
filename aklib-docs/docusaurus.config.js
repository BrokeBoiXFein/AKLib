// @ts-check
// AKLib documentation site configuration.

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AKLib',
  tagline: 'Motion & tracking library for VEX V5 (PROS)',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://aklib.example.com',
  baseUrl: '/',

  organizationName: 'aklib',
  projectName: 'aklib',

  onBrokenLinks: 'throw',

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
          routeBasePath: '/',          // docs ARE the site
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
      colorMode: {
        defaultMode: 'dark',
        respectPrefersColorScheme: true,   // follows the OS; toggle in navbar
      },
      navbar: {
        title: 'AKLib',
        items: [
          {type: 'docSidebar', sidebarId: 'docs', position: 'left', label: 'Docs'},
          {href: 'https://pros.cs.purdue.edu/v5/pros-4/index.html', label: 'PROS', position: 'right'},
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {label: 'Quickstart', to: '/getting-started/quickstart'},
              {label: 'Tutorials', to: '/tutorials/first-auton'},
              {label: 'API Reference', to: '/api/chassis'},
            ],
          },
          {
            title: 'Related',
            items: [
              {label: 'PROS', href: 'https://pros.cs.purdue.edu/v5/pros-4/index.html'},
              {label: 'Purdue SIGBots Wiki', href: 'https://wiki.purduesigbots.com/'},
              {label: 'WPILib Docs', href: 'https://docs.wpilib.org/en/stable/index.html'},
            ],
          },
        ],
        copyright: `AKLib — built for VEX V5 on PROS 4.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['cpp'],
      },
    }),
};

export default config;
