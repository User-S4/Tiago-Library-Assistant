/**:
  ros__parameters:
    motion_planner:
      disable_motion_planning: false
      planning_groups: # Sorted by order of preference
@[if has_arm_left and has_arm_right]@
        - both_arms_torso
@[elif has_arm_left and not has_arm_right]@
        - arm_left_torso
@[elif has_arm_right and not has_arm_left]@
        - arm_right_torso
@[else]@
        - torso
@[end if]@
@[if has_teleop_arms]@
        - arms_teleop
        - arm_teleop_left
        - arm_teleop_right
@[end if]@
      exclude_from_planning_joints:
        - head_1_joint
        - head_2_joint
@[if end_effector_left in ["pal-pro-gripper"]]@
        - gripper_left_finger_joint
@[end if]@
@[if end_effector_right in ["pal-pro-gripper"]]@
        - gripper_right_finger_joint
@[end if]@
      joint_tolerance: 0.01

      # Parameters for non-planned approach
      approach_velocity: 0.5
      approach_min_duration: 0.5
