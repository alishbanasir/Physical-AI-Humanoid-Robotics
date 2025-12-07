// @ts-check

/**
 * Sidebars configuration for Physical AI & Humanoid Robotics Textbook
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Module 1: ROS 2 Fundamentals
  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/ros2-architecture',
        'module-1/chapter-2',
        'module-1/chapter-3',
        'module-1/chapter-4',
      ],
    },
  ],

  // Module 2: Digital Twin Simulation
  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2/chapter-1',
        // Additional chapters will be added incrementally
      ],
    },
  ],
};

export default sidebars;
