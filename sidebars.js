// @ts-check
/**
 * Sidebars configuration for Physical AI & Humanoid Robotics Textbook
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // --- MERGED SIDEBAR (Docusaurus will display both modules in the 'Modules' dropdown) ---
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
    // --- MODULE 2 IS MERGED HERE ---
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2/chapter-1', // Chapter 1: Gazebo Physics Simulation
        'module-2/high-fidelity-rendering', // Chapter 2: Unity Rendering & HRI
        'module-2/simulating-sensors', // Chapter 3: Sensor Simulation
        'module-2/integration-best-practices', // Chapter 4: Gazebo-Unity Integration
      ],
    },
  ],
};

export default sidebars;
