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

![image](https://github.com/user-attachments/assets/b080982f-ba34-4dcc-b2e2-975c15ac230d)

Figure 1: Results of running numerous simulations timing a single robot navigating an empty room
using cooperative swarm navigation, with varying numbers of other bots assisting

The project focuses on recreating and improving the Single Robot Navigation results from the original
paper[1]. Using the ARGoS simulator, we created a swarm of Footbots that communicated
navigational data using IrRB and implemented the navigational algorithm as described by [1]. For
my initial experiment, the bots were placed in a 4 by 4-meter walled arena, and the range of
communication was limited to 1.5m to simulate a sparser swarm setup without requiring simulating a
larger area. A single target robot remains stationary in the top right corner, while a "navigator" (or
"nav") robot starts in the bottom left. The rest of the arena is filled (randomly and uniformly) with
assistant bots that wander aimlessly, turning around when they bump into obstacles. In my initial
experiment, we used the Navigation with Stopping (NwS) algorithm, where the navigator robot
would stop moving once it reached its last known navigational coordinate. The number of other
bots was changed between values of 1, 5, 10, 15, and 20, then the time it took for the navigator to
successfully find the target bot was recorded. (Figure 1)
While the larger swarms quickly converged on a navigation time of around 1000 ticks (100 seconds),
the sparser swarm navigation times compose a long-tailed distribution that sometimes took several
hours to find the target. Additionally, the algorithm was successfully shown to be able to navigate an
environment with randomly placed obstacles scattered throughout.
