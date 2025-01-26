# 2025 Robosharks Robot Code

## Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Team1157/2025_Ranger_rio/
   cd 2025_Ranger_rio/
   ```

2. **Install Dependencies**:
   - Ensure you have the latest version of WPILib installed, it comes with Elastic Dashboard
   - If running the real robot install NI game tools (message me for the licensing) 

3. **Deploy Code (Real robot only)**:
   - Connect your robot to your development machine.
   - Use your IDE (VSCode with WPILib plugin is what I use and you should too) to build and deploy the code to the robot controller.
   - OR while in the directory run `./gradlew deploy` (linux/macos) `gradlew.bat deploy` (windows)

4. **Run Simulation** *(Optional)*:
   - Open the VSCode command palette (CTRL, Shift, P), type `>WPILIB: Simulate Robot Code`, and hit enter.
   - Drag your joystick onto joystick 0
   - Put the robot into teleop
   - Drive!

## Controller Mapping

- **Left Stick (Y-axis)**: Forward/Backward movement.  
- **Left Stick (X-axis)**: Strafe Left/Right movement.  
- **Right Stick (Y-axis)**: Elevation control (up/down).  
- **Right Stick (X-axis)**: Yaw control (rotation around Z-axis).  
- **Right Bumper**: Increase Pitch (tilt up).  
- **Left Bumper**: Decrease Pitch (tilt down).  
- **Left Trigger**: Roll left (rotation around X-axis).  
- **Right Trigger**: Roll right (rotation around X-axis).  
- **A Button**: Open Newton Gripper.  
- **B Button**: Close Newton Gripper.  
- **X Button**: Calibrate Gyro.  
- **Y Button**: Reset Gyro.  

## Thruster Assignments

| PWM Port | Component              | Description                     |
|----------|------------------------|---------------------------------|
| 0        | `m_leftFront45`        | Left front 45-degree thruster   |
| 1        | `m_leftRear45`         | Left rear 45-degree thruster    |
| 2        | `m_rightFront45`       | Right front 45-degree thruster  |
| 3        | `m_rightRear45`        | Right rear 45-degree thruster   |
| 4        | `m_leftFrontForward`   | Left front vertical thruster    |
| 5        | `m_leftRearForward`    | Left rear vertical thruster     |
| 6        | `m_rightFrontForward`  | Right front vertical thruster   |
| 7        | `m_rightRearForward`   | Right rear vertical thruster    |
| 9        | `m_newtonGripper`      | Newton gripper motor            |

## Troubleshooting

1. **Robot is not responding to controller inputs**:
   - Ensure the controller is properly connected and dragged to the right port in the Driverstation
   - Make sure the robot is in teleop
     
3. **Gyro issues**:
   - Recalibrate the gyro using the X button.
   - Reset the gyro using the Y button.
   - make sure the bot is stationary while calibrating

5. **Pose data not showing correctly in AdvantageScope**:
   - Drag the robot pose from the field2d, not the pose3d into the field in the bottom
   - right click on it in the bottom field and make sure rotation is in degrees, not radians

## Degrees of Freedom Implementation

1. **Forward/Backward (Translation on X-axis)**:
   - Controlled by adjusting the power of the 45-degree thrusters (`m_leftFront45`, `m_rightFront45`, `m_leftRear45`, `m_rightRear45`) in unison. Positive values move the robot forward, while negative values move it backward.

2. **Strafe Left/Right (Translation on Y-axis)**:
   - Achieved by counter-balancing the 45-degree thrusters to generate lateral movement. For example, increasing the left-side thrusters' power while decreasing the right-side thrusters enables strafing.

3. **Up/Down (Translation on Z-axis)**:
   - Managed by the vertical thrusters (`m_leftFrontForward`, `m_rightFrontForward`, `m_leftRearForward`, `m_rightRearForward`). Increasing thrust moves the robot up, while decreasing thrust moves it down.

4. **Yaw (Rotation about Z-axis)**:
   - Controlled by applying opposite power to diagonal pairs of 45-degree thrusters (e.g., `m_leftFront45` and `m_rightRear45` vs. `m_rightFront45` and `m_leftRear45`) to rotate left or right.

5. **Pitch (Rotation about X-axis)**:
   - Adjusted using the vertical thrusters at the front (`m_leftFrontForward`, `m_rightFrontForward`) and rear (`m_leftRearForward`, `m_rightRearForward`). Increasing front thrust while decreasing rear thrust tilts the robot forward, and vice versa.

6. **Roll (Rotation about Y-axis)**:
   - Achieved by manipulating the left and right vertical thrusters. For example, increasing power on the left-side vertical thrusters and decreasing it on the right-side ones causes the robot to roll clockwise.

## Contributing
To contribute:

1. Fork the repository and create a new branch for your feature or bugfix, name the fork `<your name>_2025_Ranger_rio` ie. `ada_2025_Ranger_rio`
2. Ensure your code follows the existing style (just run it through a java beautifier) and run it in sim to make sure it doesnt crash.
3. Write clear and concise commit messages.
4. Submit a pull request with a detailed description of your changes.
5. I'll merge it into the main branch if everything is okay and it'll become a part of our robot!

Feel free to open issues or just talk to me (ada) for questions. Thank you :3

