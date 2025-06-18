import subprocess
import os
import time


def close_existing_processes():
    try:
        # Kill all gnome-terminal processes
        subprocess.run("pkill gnome-terminal", shell=True, check=False)
        # Kill all RViz processes
        subprocess.run("pkill rviz2", shell=True, check=False)
    except Exception as e:
        print(f"Failed to close existing processes: {e}")


def run_command_in_new_terminal(command):
    try:
        # Use shell=True to allow the command to be passed as a single string
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])
    except FileNotFoundError:
        print("gnome-terminal not found. Make sure it is installed on your system.")
    except Exception as e:
        print(f"Failed to open terminal: {e}")

def run_multiple_commands(commands):
    for command in commands:
        run_command_in_new_terminal(command)
    
    
commands1 = [
    "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur16e robot_ip:=169.254.56.120 kinematics_params_file:=/home/utilisateurom/Documents/ROS_WORK/my_robot_calibration.yaml headless_mode:=True",
    "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur16e launch_rviz:=true launch_servo:=true",
]
commands2 = ["ros2 control switch_controllers --deactivate forward_velocity_controller --deactivate scaled_joint_trajectory_controller --activate forward_position_controller",
             "ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType '{command_type: 1}'",

]
commands3 = ["echo 'echoing /forward_position_controller/commands'; ros2 topic echo /forward_position_controller/commands",
             "echo 'echoing /servo_node/delta_twist_cmds'; ros2 topic echo /servo_node/delta_twist_cmds"
]

if __name__ == "__main__":
    close_existing_processes()
    run_multiple_commands(commands1)
    time.sleep(5)  # Wait for the first set of commands to initialize
    run_multiple_commands(commands2)
    time.sleep(2)  # Wait for the second set of commands to initialize
    run_multiple_commands(commands3)
