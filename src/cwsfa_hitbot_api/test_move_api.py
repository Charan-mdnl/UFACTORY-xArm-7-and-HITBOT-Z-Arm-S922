import time
from hitbot_interface import HitbotInterface

def main():
    robot_id = 111
    robot = HitbotInterface(robot_id)
    print("Connecting to robot using hitbot-api...")
    robot.net_port_initial()

    time.sleep(1)
    if robot.is_connect() == 1:
        print('Robot connected successfully.')
        init = robot.initial(1, 1000)
        
        if init == 1:
            print('Robot initialized.')
            
            # Read initial position
            robot.get_scara_param()
            print(f"Initial Pos -> J1: {robot.angle1:.2f}, J2: {robot.angle2:.2f}, J3: {robot.z:.2f}, J4: {robot.r:.2f}")

            # Send movement command
            print("Moving J1 by +5 degrees and J2 by +5 degrees...")
            robot.unlock_position()
            robot.new_movej_angle(robot.angle1 + 10.0, robot.angle2 + 10.0, robot.z, robot.r, 20.0, 0.0)
            robot.wait_stop()
            
            # Read position after movement
            robot.get_scara_param()
            print(f"Final Pos   -> J1: {robot.angle1:.2f}, J2: {robot.angle2:.2f}, J3: {robot.z:.2f}, J4: {robot.r:.2f}")
            
        else:
            print('Failed to initialize robot.')
    else:
        print('No robot connection.')

if __name__ == '__main__':
    main()
