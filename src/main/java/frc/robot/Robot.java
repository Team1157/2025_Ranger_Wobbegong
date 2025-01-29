// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.BlueRoboticsBasicESC;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;

/**
 * This class controls an underwater robot with 8 thrusters configured for full 3D movement
 * and includes simulation for 3d testing in a virtual environment.
 * The robot is equipped with a Newton gripper for manipulation and completion of tasks.
 * As of now the 3D physics simulation requires a real driverstation and Xbox controller to work
 * how it does on the Robot, although WPILib simulation works fine when viewing the Field2d network
 * table widget, hence its inclusion in the code.
 */
public class Robot extends LoggedRobot {

  // Thruster definitions
  private final BlueRoboticsBasicESC m_leftFront45 = new BlueRoboticsBasicESC(0);
  private final BlueRoboticsBasicESC m_leftRear45 = new BlueRoboticsBasicESC(1);
  private final BlueRoboticsBasicESC m_rightFront45 = new BlueRoboticsBasicESC(2);
  private final BlueRoboticsBasicESC m_rightRear45 = new BlueRoboticsBasicESC(3);
  private final BlueRoboticsBasicESC m_leftFrontForward = new BlueRoboticsBasicESC(4);
  private final BlueRoboticsBasicESC m_leftRearForward = new BlueRoboticsBasicESC(5);
  private final BlueRoboticsBasicESC m_rightFrontForward = new BlueRoboticsBasicESC(6);
  private final BlueRoboticsBasicESC m_rightRearForward = new BlueRoboticsBasicESC(7);
  private final BlueRoboticsBasicESC m_newtonGripper = new BlueRoboticsBasicESC(9);
  private final XboxController m_controller = new XboxController(0);

  // Gyro definition
  private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  private SimDeviceSim m_simulatedGyro;
  private SimDouble m_simulatedGyroAngle;

  // Timer for gripper control
  private final Timer gripperTimer = new Timer();

  // For notifications
  private String lastGripperState = "Stopped";

  // Simulation-related fields
  private final Field2d m_field = new Field2d();
  private Pose3d m_pose = new Pose3d();
  private Translation3d m_velocity = new Translation3d();
  private Rotation3d m_rotation = new Rotation3d();

  // Publishers for AdvantageScope
  private StructPublisher < Pose3d > m_posePublisher;
  private StructArrayPublisher < Pose3d > m_poseArrayPublisher;

  // Toggle for pool-relative control
  private boolean m_poolRelative = false;
  private NetworkTableEntry m_poolRelativeToggle;

  public Robot() {
    // Set up thrusters in the SendableRegistry for debugging
    SendableRegistry.addChild(m_leftFront45, "LeftFront45");
    SendableRegistry.addChild(m_leftRear45, "LeftRear45");
    SendableRegistry.addChild(m_rightFront45, "RightFront45");
    SendableRegistry.addChild(m_rightRear45, "RightRear45");
    SendableRegistry.addChild(m_leftFrontForward, "LeftFrontForward");
    SendableRegistry.addChild(m_leftRearForward, "LeftRearForward");
    SendableRegistry.addChild(m_rightFrontForward, "RightFrontForward");
    SendableRegistry.addChild(m_rightRearForward, "RightRearForward");
  }

  @Override
  public void robotInit() {
    // Initialize simulation visualization
    SmartDashboard.putData("Field", m_field);
    SendableRegistry.addChild(m_newtonGripper, "NewtonGripper");
    if (isSimulation()) {
      m_simulatedGyro = new SimDeviceSim("Gyro:ADXRS450");
      m_simulatedGyroAngle = m_simulatedGyro.getDouble("Angle");
    }

    // Initialize AdvantageScope publishers
    m_posePublisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose3d.struct).publish();
    m_poseArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("RobotPoseArray", Pose3d.struct).publish();

    // Calibrate the gyro on startup
    m_imu.calibrate();
    Elastic.sendNotification(new Elastic.Notification().withLevel(Elastic.Notification.NotificationLevel.INFO).withTitle("Gyro Calibration").withDescription("Gyro calibrating on startup.").withDisplaySeconds(5.0));

    // Add pool-relative toggle to Elastic
    m_poolRelativeToggle = SmartDashboard.getEntry("PoolRelative");
    m_poolRelativeToggle.setBoolean(m_poolRelative);

    // Initialize the gripper status field in SmartDashboard for our Elastic
    SmartDashboard.putString("Gripper Status", lastGripperState);

    // Start the gripper timer
    gripperTimer.reset();
    gripperTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    // Teleop control logic (omitted here for brevity)
    // Update simulation state and AdvantageScope publishers
    // Get toggle state from Elastic
    m_poolRelative = m_poolRelativeToggle.getBoolean(false);

    // Read controller inputs with deadband applied
    double forward = applyDeadband( - m_controller.getLeftY(), 0.1); // Forward/backward
    double strafe = applyDeadband( - m_controller.getLeftX(), 0.1); // Left/right
    double vertical = applyDeadband(
    m_controller.getRightY() - m_controller.getLeftY(), 0.1); // Up/down
    double yaw = applyDeadband(
    m_controller.getRightX(), 0.1); // Yaw rotation
    double roll = (
    m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis()); // Analog roll control
    // Pitch control using bumpers
    double pitch = 0.0;
    if (m_controller.getRightBumperButton()) {
      pitch += 0.5; // Adjust pitch forward (positive value)
    }
    if (m_controller.getLeftBumperButton()) {
      pitch -= 0.5; // Adjust pitch backward (negative value)
    }

    // Field-relative transformation
    double poolX = forward;
    double poolY = strafe;
    double poolZ = vertical;

    if (m_poolRelative) {
      // Get IMU angles
      double yawAngle = Math.toRadians(m_imu.getAngle()); // Yaw (rotation around Z)
      double pitchAngle = Math.toRadians(m_imu.getGyroAngleY()); // Pitch (rotation around X)
      double rollAngle = Math.toRadians(m_imu.getGyroAngleX()); // Roll (rotation around Y)
      // Calculate rotation matrix components
      double cosYaw = Math.cos(yawAngle);
      double sinYaw = Math.sin(yawAngle);
      double cosPitch = Math.cos(pitchAngle);
      double sinPitch = Math.sin(pitchAngle);
      double cosRoll = Math.cos(rollAngle);
      double sinRoll = Math.sin(rollAngle);

      // Transformation for pool-relative controls
      double rotatedX = forward * (cosYaw * cosPitch) + strafe * (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) + vertical * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
      double rotatedY = forward * (sinYaw * cosPitch) + strafe * (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) + vertical * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);
      double rotatedZ = forward * ( - sinPitch) + strafe * (cosPitch * sinRoll) + vertical * (cosPitch * cosRoll);

      poolX = rotatedX;
      poolY = rotatedY;
      poolZ = rotatedZ;
    }

    // Set power to thrusters for 3D movement
    m_leftFront45.set(poolY + poolX);
    m_leftRear45.set( - poolY + poolX);
    m_rightFront45.set(poolY - poolX);
    m_rightRear45.set( - poolY - poolX);

    // Vertical, pitch, roll, and yaw control
    m_leftFrontForward.set(poolZ + roll + yaw + pitch); // Add pitch adjustment
    m_leftRearForward.set(poolZ - roll + yaw - pitch); // Subtract pitch adjustment
    m_rightFrontForward.set(poolZ + roll - yaw + pitch); // Add pitch adjustment
    m_rightRearForward.set(poolZ - roll - yaw - pitch); // Subtract pitch adjustment
    // Control the Newton gripper
    Elastic.Notification notification = new Elastic.Notification();

    String currentGripperState = "Stopped";
    if (m_controller.getAButton()) {
      // Open the gripper for 4 seconds
      if (gripperTimer.get() >= 4.0) {
        m_newtonGripper.set(0.0);
      } else {
        m_newtonGripper.set(1.0); // Full forward power (open)
        currentGripperState = "Opening";
      }
    } else if (m_controller.getBButton()) {
      // Close the gripper for 4 seconds
      if (gripperTimer.get() >= 4.0) {
        m_newtonGripper.set(0.0);
      } else {
        m_newtonGripper.set( - 1.0); // Full reverse power (close)
        currentGripperState = "Closing";
      }
    } else {
      // Stop the gripper
      m_newtonGripper.set(0.0);
      gripperTimer.reset();
    }

    // Notification logic for Newton gripper to send the elastic notifications on state change
    if (!currentGripperState.equals(lastGripperState)) {
      Elastic.sendNotification(notification.withLevel(Elastic.Notification.NotificationLevel.INFO).withTitle("Gripper " + currentGripperState).withDescription("Power set to: " + m_newtonGripper.getVoltage()).withDisplaySeconds(5.0));
      lastGripperState = currentGripperState;

      // Update the gripper status on SmartDashboard for Elastic
      SmartDashboard.putString("Gripper Status", currentGripperState);
    }

    // Gyro control and notifications for calibration and reset
    if (m_controller.getXButtonPressed()) {
      m_imu.calibrate();
      Elastic.sendNotification(new Elastic.Notification().withLevel(Elastic.Notification.NotificationLevel.WARNING).withTitle("Gyro Calibration").withDescription("Gyro calibrating as requested.").withDisplaySeconds(5.0));
    }

    if (m_controller.getYButtonPressed()) {
      m_imu.reset();
      Elastic.sendNotification(new Elastic.Notification().withLevel(Elastic.Notification.NotificationLevel.INFO).withTitle("Gyro Reset").withDescription("Gyro heading reset to zero.").withDisplaySeconds(5.0));
    }

    // Update simulation with pitch, roll, and yaw control inputs
    updateSimulation(poolX, poolY, vertical, yaw, pitch);
  }

  /**
   * Applies a deadband to the joystick input to filter out small, unintended movements.
   *
   * @param value    The joystick input value.
   * @param deadband The deadband threshold (applied symmetrically).
   * @return The adjusted joystick value, after the deadband is applied.
   */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    } else {
      return 0.0;
    }
  }

  /**
   * Updates the simulation state based on the current inputs and includes water resistance.
   *
   * @param x      Forward/backward input (positive forward)
   * @param y      Left/right input (positive right)
   * @param z      Up/down input (positive up)
   * @param rotate Rotation input (yaw control)
   * @param pitch  Pitch control input (positive forward)
   */
  private void updateSimulation(double x, double y, double z, double rotate, double pitch) {
    // Simulation time step (20ms)
    double deltaTime = 0.02;

    // Water resistance coefficients (drag constants)
    final double dragCoefficientLinear = 0.5; // Linear drag for x, y, z
    final double dragCoefficientRotational = 0.3; // Rotational drag for yaw (z)
    // Convert robot-relative inputs to global frame using the robot's current rotation (yaw)
    Translation3d robotRelativeForce = new Translation3d(x, y, z);
    Translation3d globalForce = robotRelativeForce.rotateBy(m_pose.getRotation());

    // Calculate rotational force (yaw is unaffected by the translation forces)
    double rotationalForce = rotate;
    double pitchForce = pitch;

    // Apply water resistance (drag force reduces velocity) 
    Translation3d dragForce = new Translation3d( - dragCoefficientLinear * m_velocity.getX(), -dragCoefficientLinear * m_velocity.getY(), -dragCoefficientLinear * m_velocity.getZ());
    double rotationalDragForce = -dragCoefficientRotational * m_rotation.getZ();

    // Update velocity with applied forces and drag (linear and rotational)
    m_velocity = new Translation3d(
    m_velocity.getX() + (globalForce.getX() + dragForce.getX()) * deltaTime, m_velocity.getY() + (globalForce.getY() + dragForce.getY()) * deltaTime, m_velocity.getZ() + (globalForce.getZ() + dragForce.getZ()) * deltaTime);

    // Update rotation rates with applied forces and drag 
    m_rotation = new Rotation3d(
    m_rotation.getX() + pitchForce * deltaTime, m_rotation.getY(), m_rotation.getZ() + (rotationalForce + rotationalDragForce) * deltaTime);

    // Update pose (position and orientation) 
    m_pose = new Pose3d(
    m_pose.getTranslation().plus(m_velocity.times(deltaTime)), new Rotation3d(
    m_pose.getRotation().getX() + m_rotation.getX() * deltaTime, m_pose.getRotation().getY() + m_rotation.getY() * deltaTime, m_pose.getRotation().getZ() + m_rotation.getZ() * deltaTime));

    // Update the simulation visualization for 2d
    m_field.setRobotPose(m_pose.toPose2d());

    // Publish the updated pose to AdvantageScope 
    Pose3d poseA = m_pose;
    m_posePublisher.set(poseA);
    m_poseArrayPublisher.set(new Pose3d[] {
      poseA
    });
  }

  @Override
  public void simulationPeriodic() {
    // Update simulation telemetry and simulated gyro
    SmartDashboard.putString("Pose", m_pose.toString());
    SmartDashboard.putString("Velocity", m_velocity.toString());
    SmartDashboard.putString("Rotation", m_rotation.toString());

    // Update simulated gyro
    if (m_simulatedGyroAngle != null) {
      double currentAngle = m_simulatedGyroAngle.get();
      double deltaTime = 0.02; // 20ms periodic update
      double newAngle = currentAngle + m_rotation.getZ() * deltaTime * (180 / Math.PI); // Convert radians/s to degrees/s
      m_simulatedGyroAngle.set(newAngle);
    }
  }

  @Override
  public void autonomousInit() {
    // Autonomous initialization code here
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous periodic code here
  }

  @Override
  public void testInit() {
    // Test initialization code here
  }

  @Override
  public void testPeriodic() {
    m_leftFront45.set(0.5);
    m_leftRear45.set(0.5);
    m_rightFront45.set(0.5);
    m_rightRear45.set(0.5);
    m_leftFrontForward.set(0.5);
    m_leftRearForward.set(0.5);
    m_rightFrontForward.set(0.5);
    m_rightRearForward.set(0.5);
    m_newtonGripper.set(0.5);
  }
}