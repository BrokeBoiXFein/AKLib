// AKLib docs sidebar.
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'getting-started/installation',
        'getting-started/quickstart',
        'getting-started/conventions',
      ],
    },
    {
      type: 'category',
      label: 'Configuration',
      items: [
        'configuration/drivetrains',
        'configuration/sensors',
        'configuration/exit-conditions',
      ],
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        'tutorials/first-auton',
        'tutorials/motions',
        'tutorials/manual-pid-tuning',
        'tutorials/auto-tuning',
        'tutorials/imu-calibration',
        'tutorials/pure-pursuit',
        'tutorials/path-planner',
        'tutorials/motion-profiling',
        'tutorials/auton-selector',
      ],
    },
    {
      type: 'category',
      label: 'API Reference',
      items: [
        'api/chassis',
        'api/motion-params',
        'api/odometry',
        'api/drivetrain',
        'api/path',
        'api/autotune',
        'api/control',
        'api/units',
        'api/selector',
      ],
    },
    'resources',
  ],
};

export default sidebars;
