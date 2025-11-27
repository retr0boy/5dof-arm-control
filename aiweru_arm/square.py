#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def draw_square():
    # 1. Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_square_python', anonymous=True)

    # 2. Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # 3. Instantiate a MoveGroupCommander for the arm
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # Settings
    move_group.allow_replanning(True)
    move_group.set_planning_time(10.0)

    # Print Info
    print("Reference frame: %s" % move_group.get_planning_frame())
    print("End effector: %s" % move_group.get_end_effector_link())

    # 4. Get Current Pose
    # We use a helper to ensure we have clean Geometry Messages
    start_pose = move_group.get_current_pose().pose
    
    waypoints = []

    # Square parameters
    # 2cm square (Reduced to ensure kinematic feasibility)
    side_length = 0.5

    # Helper to create clean Pose objects
    def create_pose(ref):
        p = geometry_msgs.msg.Pose()
        p.position.x = ref.position.x
        p.position.y = ref.position.y
        p.position.z = ref.position.z
        p.orientation.x = ref.orientation.x
        p.orientation.y = ref.orientation.y
        p.orientation.z = ref.orientation.z
        p.orientation.w = ref.orientation.w
        return p

    # --- Define Square Waypoints (RETRACTING FIRST) ---
    
    # 1. Move BACKWARD (-X) -> Retracting into safe workspace
    wpose = create_pose(start_pose)
    wpose.position.x -= side_length*2
    waypoints.append(wpose)

    # 2. Move LEFT (+Y)
    wpose = create_pose(wpose)
    wpose.position.y += side_length
    waypoints.append(wpose)

    # 3. Move FORWARD (+X)
    wpose = create_pose(wpose)
    wpose.position.x += side_length
    waypoints.append(wpose)

    # 4. Return to Start (-Y)
    wpose = create_pose(wpose)
    wpose.position.y -= side_length
    waypoints.append(wpose)

    print("Generating Cartesian path for %d waypoints..." % len(waypoints))

    # 5. Compute Cartesian Path (Robust Mode)
    plan = None
    fraction = 0.0
    step_size = 0.005 # 5mm resolution

    try:
        # Attempt 1: Standard API
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, step_size, True)
    except Exception as e:
        print("[WARN] Standard compute_cartesian_path failed: %s" % e)
        print("[INFO] Attempting fallback to C++ binding (_g)...")
        
        try:
            # Attempt 2: Direct C++ binding with explicit list cast
            (plan_stream, fraction) = move_group._g.compute_cartesian_path(list(waypoints), step_size, True)
            
            # Deserialize the result
            plan = moveit_msgs.msg.RobotTrajectory()
            plan.deserialize(plan_stream)
        except Exception as e2:
             print("[ERROR] Fallback also failed: %s" % e2)
             return

    print("Path computed with %.2f%% success." % (fraction * 100.0))

    # 6. Execute Path
    # Allow partial execution if at least 50% of the path is valid
    if fraction > 0.5:
        if fraction < 1.0:
            print("[WARN] Only computed %.2f%% of the path. Executing partial path..." % (fraction * 100.0))
        else:
            print("Executing complete path...")

        # --- FIX FOR TIMEOUTS: SLOW DOWN TRAJECTORY ---
        # The physical robot might be slower than the planned path.
        # We manually multiply the timestamps by 3.0 to give it 3x more time.
        print("Slowing down trajectory by 3x to prevent timeout...")
        new_traj = moveit_msgs.msg.RobotTrajectory()
        new_traj.joint_trajectory = plan.joint_trajectory
        
        scale_factor = 1.5
        
        for point in new_traj.joint_trajectory.points:
            point.time_from_start *= scale_factor
            # Optional: Scale velocities down to match the new timing
            # This helps keep the controller happy
            if point.velocities:
                point.velocities = [v / scale_factor for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a / (scale_factor**2) for a in point.accelerations]
        
        # Execute the slower plan
        move_group.execute(new_traj, wait=True)
    else:
        print("Path planning failed to cover enough of the trajectory.")
        print("TIP: The robot might still be at a singularity. Try moving joints 2 and 3 slightly using RViz/GUI to 'break' the straight line before running this.")

    # 7. Cleanup
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass