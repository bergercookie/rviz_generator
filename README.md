# Rviz Generator

## This is not a functional prototype but merely a work in progress.

*Rviz Generator* package deals with the generation of parametric rviz files
based on a template rviz file and a set of directives by the user on which
*displays* to spawn.

Assume that you have a sample rviz file which is configured to listen to topics
published by two different robots `robot_1` and `robot_2`. Each robot is
publishing under its corresponding namespace, that is `robot_ns_1`,
`robot_ns_2` respectively.

Such topics would include (e.g. in a multi-robot SLAM operation):
- Estimated trajectory
- Generated map
- Raw odometry trajectory
- Sensor visualization (laser data, camera image)

All of the above topics should be duplicated for each robot.

However, what happens when one of the robots is to publish in a different
namespace than the one the rviz file is configured to listen to (e.g. from
`robot_ns_1` to `robot_hostname`), or what about the case a new robot is to be
added to the configuration and rviz file has to track the messages published
under that namespace as well? The standard way would be to manually modify/add
the topics (as well as the names of the corresponding Rviz displays via the
Rviz GUI). *Rviz Generator* offers a dynamic way of defining the topics that
are written to the rviz file *in a parametric way* and launch Rviz with that
newly generated file as input.

% TODO - Add more details on the example

For a usage example of this see the
[mrpt_graphslam_2d](http://github.com/bergercookie/mrpt_slam/mrpt_graphslam_2d)
package.
