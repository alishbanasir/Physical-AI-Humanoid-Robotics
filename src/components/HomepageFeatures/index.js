import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master ROS 2 architecture, communication patterns (topics, services, actions),
        and learn to bridge Python AI agents with rclpy for intelligent robot control.
      </>
    ),
  },
  {
    title: 'Digital Twin Simulation',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Build and simulate humanoid robots using URDF modeling, Gazebo physics,
        and Unity visualization for realistic digital twin environments.
      </>
    ),
  },
  {
    title: 'AI-Robot Brain',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Integrate cutting-edge AI models with robotics using NVIDIA Isaac Sim,
        vision-language-action models, and embodied AI techniques.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
