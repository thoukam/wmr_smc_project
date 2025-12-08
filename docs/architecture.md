# Architecture ROS2 – WMR SMC Project

```text
          +--------------------+
          |  turtlebot3_gazebo |
          |  (simulation)      |
          +---------+----------+
                    |
                    | /odom  (nav_msgs/Odometry)
                    v
          +--------------------------+
          |  wmr_controller (node)   |
          |--------------------------|
          |  - NSMC / BSMC           |
          |  - génération ref        |
          |  - calcul erreurs (ex..) |
          +------+---------+---------+
                 |         |
     /cmd_vel    |         |
 (geometry_msgs/ |         |  /wmr/ref_path  (nav_msgs/Path)
      Twist)     |         +->/wmr/robot_path (nav_msgs/Path)
                 v
          +--------------------+
          | TurtleBot3 (base)  |
          |  (via gazebo)      |
          +--------------------+

Visualisation RViz2 :
  - Fixed frame : odom
  - Display Path :
      * Topic: /wmr/ref_path  (vert)
      * Topic: /wmr/robot_path (jaune)
  - TF et modèle du robot proviennent de turtlebot3_gazebo
```