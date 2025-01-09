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

/**
 * This class controls an underwater robot with 8 thrusters configured for full 3D movement
 * and includes simulation for testing in a virtual environment.
 */
public class Robot extends TimedRobot {
  // Thruster definitions
  private final PWMSparkMax m_leftFrontVertical = new PWMSparkMax(0);
  private final PWMSparkMax m_leftRearVertical = new PWMSparkMax(1);
  private final PWMSparkMax m_rightFrontVertical = new PWMSparkMax(2);
  private final PWMSparkMax m_rightRearVertical = new PWMSparkMax(3);
  private final PWMSparkMax m_leftFrontHorizontal = new PWMSparkMax(4);
  private final PWMSparkMax m_leftRearHorizontal = new PWMSparkMax(5);
  private final PWMSparkMax m_rightFrontHorizontal = new PWMSparkMax(6);
  private final PWMSparkMax m_rightRearHorizontal = new PWMSparkMax(7);

  private final XboxController m_controller = new XboxController(0);

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
    SendableRegistry.addChild(m_leftFrontVertical, "LeftFrontVertical");
    SendableRegistry.addChild(m_leftRearVertical, "LeftRearVertical");
    SendableRegistry.addChild(m_rightFrontVertical, "RightFrontVertical");
    SendableRegistry.addChild(m_rightRearVertical, "RightRearVertical");
    SendableRegistry.addChild(m_leftFrontHorizontal, "LeftFrontHorizontal");
    SendableRegistry.addChild(m_leftRearHorizontal, "LeftRearHorizontal");
    SendableRegistry.addChild(m_rightFrontHorizontal, "RightFrontHorizontal");
    SendableRegistry.addChild(m_rightRearHorizontal, "RightRearHorizontal");
  }

  @Override
  public void robotInit() {
    // Initialize simulation visualization
    SmartDashboard.putData("Field", m_field);

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

    // Calculate power for vertical thrusters for up/down and rotation
    double leftFrontVerticalPower = z + rotate;
    double leftRearVerticalPower = z + rotate;
    double rightFrontVerticalPower = z - rotate;
    double rightRearVerticalPower = z - rotate;

    // Calculate power for horizontal thrusters for forward/backward and strafing
    double leftFrontHorizontalPower = x + y;
    double leftRearHorizontalPower = x - y;
    double rightFrontHorizontalPower = x - y;
    double rightRearHorizontalPower = x + y;

    // Set power to thrusters
    m_leftFrontVertical.set(leftFrontVerticalPower);
    m_leftRearVertical.set(leftRearVerticalPower);
    m_rightFrontVertical.set(rightFrontVerticalPower);
    m_rightRearVertical.set(rightRearVerticalPower);

    m_leftFrontHorizontal.set(leftFrontHorizontalPower);
    m_leftRearHorizontal.set(leftRearHorizontalPower);
    m_rightFrontHorizontal.set(rightFrontHorizontalPower);
    m_rightRearHorizontal.set(rightRearHorizontalPower);

    // Update simulation
    updateSimulation(x, y, z, rotate);
  }

  /**
   * Applies a deadband to the joystick input to filter out small, unintended movements 
   * and shitty joysticks
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
    double[] poseData = new double[]{
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