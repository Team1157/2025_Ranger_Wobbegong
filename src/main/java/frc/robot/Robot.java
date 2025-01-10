// Copyright (c) Ada Tessar
package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;

/**
 * This class controls an underwater robot with 8 thrusters configured for full 3D movement
 * and includes simulation for testing in a virtual environment.
 */
public class Robot extends TimedRobot {
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

  //For the notifications
  private String lastGripperState = "Stopped";

  // Simulation-related fields
  private final Field2d m_field = new Field2d();
  private Pose3d m_pose = new Pose3d();
  private Translation3d m_velocity = new Translation3d();
  private Rotation3d m_rotation = new Rotation3d();

  // NetworkTables for AdvantageScope
  private NetworkTable m_advantageScopeTable;
  private NetworkTableEntry m_poseEntry;

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
    // Initialize NetworkTables for AdvantageScope
    m_advantageScopeTable = NetworkTableInstance.getDefault().getTable("AdvantageScope");
    m_poseEntry = m_advantageScopeTable.getEntry("Pose3d");
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void teleopPeriodic() {
      // Get input values from the controller and apply deadbands
      double x = applyDeadband(-m_controller.getLeftY(), 0.1); // Forward/backward (±x)
      double y = applyDeadband(-m_controller.getRightX(), 0.1); // Left/right (±y)
      double z = applyDeadband(-m_controller.getRightY(), 0.1); // Up/down (±z)
      double rotate = applyDeadband(-m_controller.getLeftX(), 0.1); // Rotation
  
      // Set power to thrusters
      m_leftFrontForward.set(x);
      m_leftRearForward.set(x);
      m_rightFrontForward.set(x);
      m_rightRearForward.set(x);
  
      m_leftFront45.set(y + rotate);
      m_leftRear45.set(-y + rotate);
      m_rightFront45.set(y - rotate);
      m_rightRear45.set(-y - rotate);
  
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
  
      // Notification logic
      if (!currentGripperState.equals(lastGripperState)) {
          Elastic.sendNotification(notification
              .withLevel(Elastic.Notification.NotificationLevel.INFO)
              .withTitle("Gripper " + currentGripperState)
              .withDescription("Power set to: " + m_newtonGripper.getVoltage())
              .withDisplaySeconds(5.0)
          );
          lastGripperState = currentGripperState;
      }
  
      // Update simulation
      updateSimulation(x, y, z, rotate);
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
   */
  private void updateSimulation(double x, double y, double z, double rotate) {
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
      m_rotation.getX(),
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