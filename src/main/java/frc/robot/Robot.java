// Copyright (c) Ada Tessar
package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;

/**
 * This class controls an underwater robot with 8 thrusters configured for full 3D movement
 * and includes simulation for testing in a virtual environment.
 */
public class Robot extends LoggedRobot {

  // Thruster definitions
  private final PWMSparkMax m_leftFront45 = new PWMSparkMax(0);
  private final PWMSparkMax m_leftRear45 = new PWMSparkMax(1);
  private final PWMSparkMax m_rightFront45 = new PWMSparkMax(2);
  private final PWMSparkMax m_rightRear45 = new PWMSparkMax(3);
  private final PWMSparkMax m_leftFrontForward = new PWMSparkMax(4);
  private final PWMSparkMax m_leftRearForward = new PWMSparkMax(5);
  private final PWMSparkMax m_rightFrontForward = new PWMSparkMax(6);
  private final PWMSparkMax m_rightRearForward = new PWMSparkMax(7);
  private final PWMSparkMax m_newtonGripper = new PWMSparkMax(9);
  private final XboxController m_controller = new XboxController(0);

  // Gyro definition
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private SimDeviceSim m_simulatedGyro;
  private SimDouble m_simulatedGyroAngle;
  
  // For notifications
  private String lastGripperState = "Stopped";

  // Simulation-related fields
  private final Field2d m_field = new Field2d();
  private Pose3d m_pose = new Pose3d();
  private Translation3d m_velocity = new Translation3d();
  private Rotation3d m_rotation = new Rotation3d();

  // NetworkTables for AdvantageScope
  private NetworkTable m_advantageScopeTable;
  private NetworkTableEntry m_poseEntry;

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
    // Initialize NetworkTables for AdvantageScope
    m_advantageScopeTable = NetworkTableInstance.getDefault().getTable("AdvantageScope");
    m_poseEntry = m_advantageScopeTable.getEntry("Pose3d");
    SmartDashboard.putData("Field", m_field);

    // Calibrate the gyro on startup
    m_gyro.calibrate();
    Elastic.sendNotification(new Elastic.Notification()
        .withLevel(Elastic.Notification.NotificationLevel.INFO)
        .withTitle("Gyro Calibration")
        .withDescription("Gyro calibrating on startup.")
        .withDisplaySeconds(5.0));

    // Add pool-relative toggle to Shuffleboard
    m_poolRelativeToggle = SmartDashboard.getEntry("PoolRelative");
    m_poolRelativeToggle.setBoolean(m_poolRelative);
  }

  @Override
  public void teleopPeriodic() {
      // Get the current toggle state from Shuffleboard
      m_poolRelative = m_poolRelativeToggle.getBoolean(false);
  
      // Get input values from the controller and apply deadbands
      double forward = applyDeadband(-m_controller.getLeftY(), 0.1); // Forward/backward (±x)
      double strafe = applyDeadband(m_controller.getLeftX(), 0.1);    // Left/right (±y)
      double pitch = applyDeadband(m_controller.getRightY(), 0.1);    // Pitch control (±pitch)
  
      // Adjust vertical using right and left triggers
      double rightTrigger = m_controller.getRightTriggerAxis(); // Upward movement
      double leftTrigger = m_controller.getLeftTriggerAxis();   // Downward movement
      double vertical = applyDeadband(rightTrigger - leftTrigger, 0.1); // Combine triggers
  
      double poolX = forward;
      double poolY = strafe;
  
      if (m_poolRelative) {
          // Get gyro angle and convert robot-relative to pool-relative motion
          double gyroAngle = Math.toRadians(m_gyro.getAngle());
          double cosAngle = Math.cos(gyroAngle);
          double sinAngle = Math.sin(gyroAngle);
  
          // Convert to pool-relative directions
          poolX = forward * cosAngle - strafe * sinAngle;
          poolY = forward * sinAngle + strafe * cosAngle;
      }
  
      // Set power to thrusters for 2D movement
      m_leftFront45.set(poolY + poolX);
      m_leftRear45.set(-poolY + poolX);
      m_rightFront45.set(poolY - poolX);
      m_rightRear45.set(-poolY - poolX);
  
      // Control vertical movement and pitch
      m_leftFrontForward.set(vertical + pitch);
      m_leftRearForward.set(vertical - pitch);
      m_rightFrontForward.set(vertical + pitch);
      m_rightRearForward.set(vertical - pitch);
  
      // Control the Newton gripper
      Elastic.Notification notification = new Elastic.Notification();
  
      String currentGripperState = "Stopped";
      if (m_controller.getAButton()) {
          // Open the gripper
          m_newtonGripper.set(1.0); // Full forward power
          currentGripperState = "Opening";
      } else if (m_controller.getBButton()) {
          // Close the gripper
          m_newtonGripper.set(-1.0); // Full reverse power
          currentGripperState = "Closing";
      } else {
          // Stop the gripper
          m_newtonGripper.set(0.0);
      }
  
      // Notification logic for Newton gripper
      if (!currentGripperState.equals(lastGripperState)) {
          Elastic.sendNotification(notification
              .withLevel(Elastic.Notification.NotificationLevel.INFO)
              .withTitle("Gripper " + currentGripperState)
              .withDescription("Power set to: " + m_newtonGripper.getVoltage())
              .withDisplaySeconds(5.0));
          lastGripperState = currentGripperState;
      }
  
      // Gyro control and notifications
      if (m_controller.getXButtonPressed()) {
          m_gyro.calibrate();
          Elastic.sendNotification(new Elastic.Notification()
              .withLevel(Elastic.Notification.NotificationLevel.WARNING)
              .withTitle("Gyro Calibration")
              .withDescription("Gyro calibrating as requested.")
              .withDisplaySeconds(5.0));
      }
  
      if (m_controller.getYButtonPressed()) {
          m_gyro.reset();
          Elastic.sendNotification(new Elastic.Notification()
              .withLevel(Elastic.Notification.NotificationLevel.INFO)
              .withTitle("Gyro Reset")
              .withDescription("Gyro heading reset to zero.")
              .withDisplaySeconds(5.0));
      }
  
      // Update simulation with pitch control
      updateSimulation(poolX, poolY, vertical, 0.0, pitch);
  }
  
  

  /**
   * Applies a deadband to the joystick input to filter out small, unintended movements.
   *
   * @param value    The joystick input value.
   * @param deadband The deadband threshold.
   * @return The adjusted joystick value.
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
   * @param x      Forward/backward input
   * @param y      Left/right input
   * @param z      Up/down input
   * @param rotate Rotation input
   * @param pitch  Pitch control input
   */
  private void updateSimulation(double x, double y, double z, double rotate, double pitch) {
    // Time step (assuming teleopPeriodic is called every 20ms)
    double deltaTime = 0.02;

    // Water resistance coefficients (drag constants)
    final double dragCoefficientLinear = 0.5; // Linear drag for x, y, z
    final double dragCoefficientRotational = 0.3; // Rotational drag for yaw

    // Convert robot-relative inputs to global frame using the robot's current rotation
    Translation3d robotRelativeForce = new Translation3d(x, y, z);
    Translation3d globalForce = robotRelativeForce.rotateBy(m_pose.getRotation());

    // Calculate rotational force (yaw is unaffected by the translation forces)
    double rotationalForce = rotate;
    double pitchForce = pitch;

    // Apply water resistance (drag force reduces velocity)
    Translation3d dragForce = new Translation3d(
      -dragCoefficientLinear * m_velocity.getX(),
      -dragCoefficientLinear * m_velocity.getY(),
      -dragCoefficientLinear * m_velocity.getZ()
    );
    double rotationalDragForce = -dragCoefficientRotational * m_rotation.getZ();

    // Update velocity with applied forces and drag
    m_velocity = new Translation3d(
      m_velocity.getX() + (globalForce.getX() + dragForce.getX()) * deltaTime,
      m_velocity.getY() + (globalForce.getY() + dragForce.getY()) * deltaTime,
      m_velocity.getZ() + (globalForce.getZ() + dragForce.getZ()) * deltaTime
    );

    // Update rotation rates with applied forces and drag
    m_rotation = new Rotation3d(
      m_rotation.getX() + pitchForce * deltaTime,
      m_rotation.getY(),
      m_rotation.getZ() + (rotationalForce + rotationalDragForce) * deltaTime
    );

    // Update pose (position and orientation)
    m_pose = new Pose3d(
      m_pose.getTranslation().plus(m_velocity.times(deltaTime)),
      new Rotation3d(
        m_pose.getRotation().getX() + m_rotation.getX() * deltaTime,
        m_pose.getRotation().getY() + m_rotation.getY() * deltaTime,
        m_pose.getRotation().getZ() + m_rotation.getZ() * deltaTime
      )
    );

    // Update the simulation visualization
    m_field.setRobotPose(m_pose.toPose2d());
  }

  private void publishPoseToAdvantageScope() {
    // Convert Pose3d data into a double array format
    double[] poseData = new double[] {
      m_pose.getTranslation().getX(),
      m_pose.getTranslation().getY(),
      m_pose.getTranslation().getZ(),
      m_pose.getRotation().getX(),
      m_pose.getRotation().getY(),
      m_pose.getRotation().getZ()
    };

    // Publish the pose data
    m_poseEntry.setDoubleArray(poseData);
  }

  @Override
  public void simulationPeriodic() {
    // Publish pose to AdvantageScope
    publishPoseToAdvantageScope();

    // Update simulation telemetry
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
    // Test periodic code here
  }
}