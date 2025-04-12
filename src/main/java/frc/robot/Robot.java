// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.util.Elastic;

// My special sauce
import frc.robot.commands.CloseGripperCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.OpenGripperCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PhSensorSubsystem;
import frc.robot.subsystems.SimulationSubsystem;

/**
 * This class controls an underwater robot with 8 BlueRobotics T200 thrusters configured for full 3D 
 * movement, and includes simulation for 3d testing in a virtual environment.
 * The robot is equipped with a Newton gripper for manipulation and completion of tasks.
 * As of now the 3D physics simulation requires a real driverstation and Xbox controller to work
 * how it does on the Robot, although WPILib simulation works fine when viewing the Field2d network
 * table widget, hence its inclusion in the code.
 */
public class Robot extends LoggedRobot {
  // Controller
  private final XboxController m_controller = new XboxController(0);
  
  // Subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  private final SimulationSubsystem m_simulationSubsystem = new SimulationSubsystem();
  private final PhSensorSubsystem m_phSensorSubsystem = new PhSensorSubsystem(1);
  
  // Toggle for pool-relative control
  private boolean m_poolRelative = false;
  private NetworkTableEntry m_poolRelativeToggle;

  // Field for visualization
  private final Field2d m_field = new Field2d();
  
  // Publishers for AdvantageScope
  private BooleanPublisher m_poolRelativePublisher;
  private DoubleArrayPublisher m_controllerInputsPublisher;
  private StringPublisher m_robotStatePublisher;
  private DoublePublisher m_loopTimePublisher;
  private Timer m_loopTimer = new Timer();
  
  @Override
  public void robotInit() {
    // Set up AdvantageKit logging
    Logger.recordMetadata("ProjectName", "Wobbegong The 2025 BHS MATE ROV");
    Logger.recordMetadata("BuildDate", "2025-04-01_" + System.currentTimeMillis());
    Logger.recordMetadata("GitSHA", "Not set up");
    Logger.recordMetadata("GitDate", "Not set up");
    Logger.recordMetadata("GitBranch", "Not set up");
    Logger.recordMetadata("UseMatlab", "False");
    
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
      Logger.addDataReceiver(new NT4Publisher());
      
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Start logging
    Logger.start();
    
    // Initialize NetworkTable publishers for AdvantageScope
    initializeNetworkTablePublishers();
        
    // Initialize simulation visualization
    SmartDashboard.putData("Field", m_field);
    
    // Initialize pool-relative toggle
    m_poolRelativeToggle = SmartDashboard.getEntry("PoolRelative");
    m_poolRelativeToggle.setBoolean(m_poolRelative);
    
    // Initialize the gripper status field in SmartDashboard for Elastic
    SmartDashboard.putString("Gripper Status", "Stopped");
    
    // Set default commands
    configureDefaultCommands();
    
    // Configure button bindings
    configureButtonBindings();
    
    // Calibrate the gyro on startup
    m_driveSubsystem.calibrateGyro();
    Elastic.sendNotification(
        new Elastic.Notification()
            .withLevel(Elastic.Notification.NotificationLevel.INFO)
            .withTitle("Gyro Calibration")
            .withDescription("Gyro calibrating on startup.")
            .withDisplaySeconds(5.0)
    );
    
    // Start loop timer
    m_loopTimer.reset();
    m_loopTimer.start();
  }

  private void initializeNetworkTablePublishers() {
    // Get NetworkTable instance
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    
    // Create publishers for various robot states
    m_poolRelativePublisher = nt.getBooleanTopic("/Robot/PoolRelative").publish();
    m_controllerInputsPublisher = nt.getDoubleArrayTopic("/Robot/ControllerInputs").publish();
    m_robotStatePublisher = nt.getStringTopic("/Robot/State").publish();
    m_loopTimePublisher = nt.getDoubleTopic("/Robot/LoopTime").publish();
  }

  private void configureDefaultCommands() {
    // Set the default drive command that will run whenever no other command is using the drive subsystem
    m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_driveSubsystem,
        m_simulationSubsystem,
        () -> -applyDeadband(m_controller.getLeftY(), 0.1),  // Forward/backward
        () -> -applyDeadband(m_controller.getLeftX(), 0.1),  // Left/right
        () -> applyDeadband(m_controller.getRightY(), 0.1),  // Up/down
        () -> applyDeadband(m_controller.getRightX(), 0.1),  // Yaw rotation
        () -> m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis(), // Roll
        () -> {
            // Calculate pitch based on bumpers
            double pitch = 0.0;
            if (m_controller.getRightBumperButton()) {
                pitch += 0.5;
            }
            if (m_controller.getLeftBumperButton()) {
                pitch -= 0.5;
            }
            return pitch;
        },
        () -> m_poolRelativeToggle.getBoolean(false)
    ));
    
    // Set default simulation command
    m_simulationSubsystem.setDefaultCommand(
        new RunCommand(() -> {
            // Update the field visualization with the latest pose
            m_field.setRobotPose(m_simulationSubsystem.getPose().toPose2d());
        }, m_simulationSubsystem)
    );
  }
  
  private void configureButtonBindings() {
    // X button - Calibrate gyro
    new edu.wpi.first.wpilibj2.command.button.JoystickButton(m_controller, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSubsystem.calibrateGyro();
            Elastic.sendNotification(
                new Elastic.Notification()
                    .withLevel(Elastic.Notification.NotificationLevel.WARNING)
                    .withTitle("Gyro Calibration")
                    .withDescription("Gyro calibrating as requested.")
                    .withDisplaySeconds(5.0)
            );
        }));
    
    // Y button - Reset gyro
    new edu.wpi.first.wpilibj2.command.button.JoystickButton(m_controller, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> {
            m_driveSubsystem.resetGyro();
            Elastic.sendNotification(
                new Elastic.Notification()
                    .withLevel(Elastic.Notification.NotificationLevel.INFO)
                    .withTitle("Gyro Reset")
                    .withDescription("Gyro heading reset to zero.")
                    .withDisplaySeconds(5.0)
            );
        }));
    
    // A button - Open gripper
    new edu.wpi.first.wpilibj2.command.button.JoystickButton(m_controller, XboxController.Button.kA.value)
        .onTrue(new OpenGripperCommand(m_gripperSubsystem));
    
    // B button - Close gripper
    new edu.wpi.first.wpilibj2.command.button.JoystickButton(m_controller, XboxController.Button.kB.value)
        .onTrue(new CloseGripperCommand(m_gripperSubsystem));
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

  @Override
  public void robotPeriodic() {
    // Calculate loop time
    double loopTime = m_loopTimer.get();
    m_loopTimer.reset();
    
    // Run the CommandScheduler - this is required for the command-based framework
    CommandScheduler.getInstance().run();
    
    // Update pool-relative mode from dashboard
    m_poolRelative = m_poolRelativeToggle.getBoolean(false);
    
    // Log data to NetworkTables for AdvantageScope
    updateTelemetry(loopTime);
  }
  
  private void updateTelemetry(double loopTime) {
    // Publish loop time
    m_loopTimePublisher.set(loopTime);
    
    // Publish pool-relative state
    m_poolRelativePublisher.set(m_poolRelative);
    // Publish pH to nt
    SmartDashboard.putNumber("pH Value", m_phSensorSubsystem.getPh());

    // Publish controller inputs as an array
    m_controllerInputsPublisher.set(new double[] {
        m_controller.getLeftX(),
        m_controller.getLeftY(),
        m_controller.getRightX(),
        m_controller.getRightY(),
        m_controller.getLeftTriggerAxis(),
        m_controller.getRightTriggerAxis(),
        m_controller.getRightBumper() ? 1.0 : 0.0,
        m_controller.getLeftBumper() ? 1.0 : 0.0
    });
    
    // Publish robot state
    String currentState = "Disabled";
    if (isAutonomous()) {
        currentState = "Autonomous";
    } else if (isTeleop()) {
        currentState = "Teleop";
    } else if (isTest()) {
        currentState = "Test";
    }
    m_robotStatePublisher.set(currentState);
    
    }

  @Override
  public void simulationPeriodic() {
    // Run the simulation periodic logic in the simulation subsystem
    m_simulationSubsystem.updateSimulation();
  }

  @Override
  public void autonomousInit() {
    m_robotStatePublisher.set("Autonomous");
  }

  @Override
  public void teleopInit() {
    m_robotStatePublisher.set("Teleop");
    // Nothing specific needed here as default commands handle teleop
  }

  @Override
  public void testInit() {
    m_robotStatePublisher.set("Test");
    // Cancel all running commands
    CommandScheduler.getInstance().cancelAll();
    
    // Schedule the test command
    CommandScheduler.getInstance().schedule(new RunCommand(() -> {
        m_driveSubsystem.testThrusters();
        m_gripperSubsystem.testGripper();
    }));
  }

  @Override
  public void disabledInit() {
    m_robotStatePublisher.set("Disabled");
    // Stop all motors when disabled
    m_driveSubsystem.stopAll();
    m_gripperSubsystem.stop();
  }
}