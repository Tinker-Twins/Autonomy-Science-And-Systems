There are 3 cascaded tasks in the capstone project:

* Task 1: Wall following & obstacle avoidance
          - The robot starts here. It must successfully follow the wall and avoid the obstacles until it reaches the yellow line.

* Task 2: Line following & traffic sign detection
          - The robot must successfully detect & follow the yellow line.
          - While following the yellow line, the the robot should stop at the stop sign for 3 seconds before continuing.

* Task 3: AprilTag tracking
          - Spawn another robot in the environment past the yellow line.
          - Attach an AprilTag marker onto this robot. This robot should be teleoperated.
          - Use the namespace concept to send seperate `/cmd_vel` commands to both the robots.
          - The preceding (autonomous) robot should follow the leading (teleoperated) robot with AprilTag marker.