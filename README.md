This project is about cooperative navigation in robotics swarms. The presence of many robots in an
environment presents unique challenges and opportunities for navigation and can allow for navigation
without reliance on pre-existing maps or external guidance systems. We focus on a scenario where a
single robot must locate and navigate toward a target within an environment of randomly moving
robots going about other tasks but assisting in navigation. First, we set up a communication-based
navigation system that allows us to exchange navigation information among robots, facilitating the
searcher’s task. To achieve this, we use robots equipped with Infrared Range and Bearing (IrRB)
sensors, which allow the transition of information over line-of-sight while also knowing the distance
and direction the message was sent from. The navigation information is stored in a data structure
called a ‘navTable’ within the robot. Each entry in this table represents a target and includes the
age of the information and an estimated distance to that target. This allows robots to keep track of
and update their knowledge about the targets’ locations based on new information received from
other robots in the swarm. Since the information is transmitted over line-of-sight, it guarantees an
obstacle-free path, and a robot can follow the chain of information back to the target The problem
we are solving showcases the system’s ability to function without any type of mappings, but merely
using wireless communication for real-time data sharing, which is crucial for making decisions in
unstructured environments. By using a system that relies on real-time communication rather than a
fixed map, robots can adapt to dynamic environments, and work together without having to coordinate
complex mapping data.

![image](https://github.com/user-attachments/assets/45fd03d6-f551-4c96-bfc1-e4591df072d1)
