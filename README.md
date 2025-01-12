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
- **Right Stick (Y-axis)**: Pitch control.
- **Right Stick (X-axis)**: Roll control.
- **Left Bumper**: Rotate left (yaw).
- **Right Bumper**: Rotate right (yaw).
- **Left Trigger**: Move downwards.
- **Right Trigger**: Move upwards.
- **A Button**: Open Newton Gripper.
- **B Button**: Close Newton Gripper.
- **X Button**: Calibrate Gyro.
- **Y Button**: Reset Gyro.

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

## Contributing
To contribute:

1. Fork the repository and create a new branch for your feature or bugfix, name the fork `<your name>_2025_Ranger_rio` ie. `ada_2025_Ranger_rio`
2. Ensure your code follows the existing style (just run it through a java beautifier) and run it in sim to make sure it doesnt crash.
3. Write clear and concise commit messages.
4. Submit a pull request with a detailed description of your changes.
5. I'll merge it into the main branch if everything is okay and it'll become a part of our robot!

Feel free to open issues for questions. Thank you :3

