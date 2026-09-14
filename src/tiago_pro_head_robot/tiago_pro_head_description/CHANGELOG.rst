^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_pro_head_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.0 (2026-06-04)
-------------------
* comment camera for head ros2 control mj simulation
* fix argument
* add check for the values of the sim_type argument
* add missing sim_type parameter
* use value in defined in property for effort in mj
* set mj_ros2_control tag dependent on the sim_type
* add the mujoco default tag
* fix the parameter name to world_name and not world
* add transmission also in mujoco simulation
* create a xacro file with joint properties
* pass sim_type argument as xacro parameter
* move mj_tags to the head.urdf.xacro
* added mujoco ros2 control
* added mujoco description
* added mujoco args
* Contributors: Ortisa Poci

1.11.0 (2026-05-20)
-------------------
* added d435i option
* Contributors: martinaannicelli

1.10.1 (2026-05-20)
-------------------
* Update head.urdf.xacro
* Added urdf parent also for camera d435i
* Contributors: ileniaperrella, martinaannicelli

1.10.0 (2026-04-13)
-------------------
* Add gazebo ignition for TIAGo Pro Head
* Contributors: thomas.peyrucain

1.9.1 (2026-04-10)
------------------

1.9.0 (2025-12-15)
------------------
* Unifying frames
* adding depth frame for calibration
* Contributors: silviamasiello

1.8.0 (2025-12-05)
------------------
* Add gazebo_version xacro argument
* Contributors: Noel Jimenez

1.7.1 (2025-11-18)
------------------
* new frame for wbc addedd
* Contributors: michelacavuoto

1.7.0 (2025-10-23)
------------------

1.6.1 (2025-10-17)
------------------

1.6.0 (2025-08-27)
------------------
* remove realsense overlay
* Contributors: antoniobrandi

1.5.0 (2025-06-18)
------------------

1.4.2 (2025-03-25)
------------------

1.4.1 (2025-02-26)
------------------
* Put back old eps because this should be fixed for heads after the first one
* Add Realsense D455 to tiago pro head
* Update tiago_pro_head.urdf.xacro
* Fix head_2_link oriontation
* Fix and reduce collision meshhes
* Fix screen link + base_link + update link_2 position stl
* Increase the safety factor due to the cable getting compressed at joint limits
* Remove unecessary link + fix base_link inertia + update collision meshes + add new head_base_link meshes
* Add libgazebo_ros_video plugin to TIAGo Pro Head
* Contributors: thomas.peyrucain, thomaspeyrucain

1.4.0 (2025-01-16)
------------------
* Merge branch 'tpe/simplify-3d-model' into 'humble-devel'
  Add simplyfied models
  See merge request robots/tiago_pro_head_robot!13
* revert collision mesh
* Add simplyfied models
* Contributors: thomas.peyrucain, thomaspeyrucain

1.3.0 (2025-01-08)
------------------

1.2.0 (2024-12-02)
------------------

1.1.0 (2024-12-02)
------------------
* Rotate sellion link
* Contributors: Aina

1.0.0 (2024-11-29)
------------------
* Update module priority
* Contributors: Aina

0.0.2 (2024-11-28)
------------------

0.0.1 (2024-11-18)
------------------
* Add condition for cameras in urdf
* Fix path for ros2 controller gazebo cfg file
* Add gazebo plugin in ros2_control
* Update pm2
* Fix control system name
* Update dependency from xacro
* Add proper collision files for the base
* Align with tiago_pro fix on the origins of the base links for the head
* Add camera argument
* Add xacro tests
* Add proper urdfs to description pkg & ros2 control
* Change official name
* Contributors: Aina, davidterkuile
