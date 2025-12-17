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
    // --- MODULE 3: THE AI-ROBOT BRAIN (NVIDIA ISAAC™) ---
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      link: {
        type: 'doc',
        id: 'textbook/module-3-ai-robot-brain/index',
      },
      collapsible: true,
      collapsed: false,
      items: [
        // Chapter 1: NVIDIA Isaac Sim Setup
        {
          type: 'category',
          label: 'Chapter 1: Isaac Sim Setup',
          link: {
            type: 'doc',
            id: 'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/index',
          },
          collapsible: true,
          collapsed: false,
          items: [
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-1-architecture',
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-2-installation',
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-3-urdf-to-usd',
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-4-scene-creation',
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-5-ros2-integration',
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-6-python-scripting',
            'textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/1-7-exercises',
          ],
        },
        // Chapter 2: Hardware-Accelerated Perception (Isaac ROS, VSLAM)
        {
          type: 'doc',
          id: 'textbook/module-3-ai-robot-brain/chapter-2-isaac-ros-perception/index',
          label: 'Chapter 2: Isaac ROS Perception',
        },
        // Chapter 3: Reinforcement Learning (RL) Training
        {
          type: 'doc',
          id: 'textbook/module-3-ai-robot-brain/chapter-3-rl-training/index',
          label: 'Chapter 3: RL Training',
        },
        // Chapter 4: Humanoid Navigation (Nav2 Integration)
        {
          type: 'doc',
          id: 'textbook/module-3-ai-robot-brain/chapter-4-nav2-integration/index',
          label: 'Chapter 4: Nav2 Integration',
        },
      ],
    },
    // --- MODULE 4: VISION-LANGUAGE-ACTION (VLA) ---
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'doc',
        id: 'textbook/module-4-vla/index',
      },
      collapsible: true,
      collapsed: false,
      items: [
        // Chapter 1: Voice-to-Action Pipeline with OpenAI Whisper
        {
          type: 'doc',
          id: 'textbook/module-4-vla/chapter-1-voice-to-action/index',
          label: 'Chapter 1: Voice-to-Action Pipeline',
        },
        // Chapter 2: Cognitive Planning with LLMs
        {
          type: 'doc',
          id: 'textbook/module-4-vla/chapter-2-cognitive-planning/index',
          label: 'Chapter 2: Cognitive Planning with LLMs',
        },
        // Chapter 3: Capstone Project - Autonomous Humanoid
        {
          type: 'doc',
          id: 'textbook/module-4-vla/chapter-3-capstone-project/index',
          label: 'Chapter 3: Capstone Project',
        },
        // Chapter 4: Advanced VLA & Real-World Deployment
        {
          type: 'doc',
          id: 'textbook/module-4-vla/chapter-4-advanced-deployment/index',
          label: 'Chapter 4: Advanced VLA & Deployment',
        },
      ],
    },
  ],
};

export default sidebars;

