// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationSubsystem extends SubsystemBase {
    // Simulation-related fields
    private Pose3d m_pose = new Pose3d();
    private Translation3d m_velocity = new Translation3d();
    private Rotation3d m_rotation = new Rotation3d();
    
    // Simulation time step
    private final double m_deltaTimeSeconds = 0.02; // 20ms
    
    // Publishers for AdvantageScope
    private StructPublisher<Pose3d> m_posePublisher;
    private StructArrayPublisher<Pose3d> m_poseArrayPublisher;
    
    // Simulated gyro
    private SimDeviceSim m_simulatedGyro;
    private SimDouble m_simulatedGyroAngle;
    
    // Current inputs
    private double m_simXMetersPerSecond = 0.0;
    private double m_simYMetersPerSecond = 0.0;
    private double m_simZMetersPerSecond = 0.0;
    private double m_simRotateRadiansPerSecond = 0.0;
    private double m_simPitchRadiansPerSecond = 0.0;
    
    public SimulationSubsystem() {
        // Initialize AdvantageScope publishers
        m_posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("RobotPose", Pose3d.struct)
                .publish();
                
        m_poseArrayPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("RobotPoseArray", Pose3d.struct)
                .publish();
        
        // Initialize simulated gyro if in simulation
        if (shouldSimulate()) {
            m_simulatedGyro = new SimDeviceSim("Gyro:ADXRS450");
            m_simulatedGyroAngle = m_simulatedGyro.getDouble("Angle");
        }
    }
    
    /**
     * Check if we should simulate.
     *
     * @return True if in simulation mode
     */
    private boolean shouldSimulate() {
        try {
            return edu.wpi.first.wpilibj.RobotBase.isSimulation();
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Set the simulation inputs using raw values.
     *
     * @param xMetersPerSecond      Forward/backward input (meters per second)
     * @param yMetersPerSecond      Left/right input (meters per second)
     * @param zMetersPerSecond      Up/down input (meters per second)
     * @param rotateRadiansPerSecond Yaw rotation input (radians per second)
     * @param pitchRadiansPerSecond  Pitch rotation input (radians per second)
     */
    public void setInputs(double xMetersPerSecond, double yMetersPerSecond, double zMetersPerSecond, 
                         double rotateRadiansPerSecond, double pitchRadiansPerSecond) {
        m_simXMetersPerSecond = xMetersPerSecond;
        m_simYMetersPerSecond = yMetersPerSecond;
        m_simZMetersPerSecond = zMetersPerSecond;
        m_simRotateRadiansPerSecond = rotateRadiansPerSecond;
        m_simPitchRadiansPerSecond = pitchRadiansPerSecond;
    }
    
    /**
     * Set the simulation inputs using different units.
     *
     * @param xFeetPerSecond      Forward/backward input (feet per second)
     * @param yFeetPerSecond      Left/right input (feet per second)
     * @param zFeetPerSecond      Up/down input (feet per second)
     * @param rotateDegreesPerSecond Yaw rotation input (degrees per second)
     * @param pitchDegreesPerSecond  Pitch rotation input (degrees per second)
     */
    public void setInputsWithUnits(double xFeetPerSecond, double yFeetPerSecond, double zFeetPerSecond, 
                                  double rotateDegreesPerSecond, double pitchDegreesPerSecond) {
        m_simXMetersPerSecond = Units.feetToMeters(xFeetPerSecond);
        m_simYMetersPerSecond = Units.feetToMeters(yFeetPerSecond);
        m_simZMetersPerSecond = Units.feetToMeters(zFeetPerSecond);
        m_simRotateRadiansPerSecond = Units.degreesToRadians(rotateDegreesPerSecond);
        m_simPitchRadiansPerSecond = Units.degreesToRadians(pitchDegreesPerSecond);
    }
    
    /**
     * Get the current pose.
     *
     * @return The current simulated pose
     */
    public Pose3d getPose() {
        return m_pose;
    }
    
    /**
     * Update the simulation state.
     */
    public void updateSimulation() {
        if (!shouldSimulate()) {
            return;
        }
        
        // Water resistance coefficients (drag constants)
        final double dragCoefficientLinear = 2.9; // Linear drag for x, y, z
        final double dragCoefficientRotational = 2.8; // Rotational drag for yaw (z)
        
        // Convert robot-relative inputs to global frame using the robot's current rotation (yaw)
        Translation3d robotRelativeForce = new Translation3d(
            m_simXMetersPerSecond,
            m_simYMetersPerSecond,
            m_simZMetersPerSecond
        );
        Translation3d globalForce = robotRelativeForce.rotateBy(m_pose.getRotation());

        // Calculate rotational force (yaw is unaffected by the translation forces)
        double rotationalForce = m_simRotateRadiansPerSecond;
        double pitchForce = m_simPitchRadiansPerSecond;

        // Apply water resistance (drag force reduces velocity) 
        Translation3d dragForce = new Translation3d(
            -dragCoefficientLinear * m_velocity.getX(),
            -dragCoefficientLinear * m_velocity.getY(),
            -dragCoefficientLinear * m_velocity.getZ()
        );
        
        double rotationalDragForce = -dragCoefficientRotational * m_rotation.getZ();

        // Update velocity with applied forces and drag (linear and rotational)
        m_velocity = new Translation3d(
            m_velocity.getX() + (globalForce.getX() + dragForce.getX()) * m_deltaTimeSeconds,
            m_velocity.getY() + (globalForce.getY() + dragForce.getY()) * m_deltaTimeSeconds,
            m_velocity.getZ() + (globalForce.getZ() + dragForce.getZ()) * m_deltaTimeSeconds
        );

        // Update rotation rates with applied forces and drag 
        m_rotation = new Rotation3d(
            m_rotation.getX() + pitchForce * m_deltaTimeSeconds,
            m_rotation.getY(),
            m_rotation.getZ() + (rotationalForce + rotationalDragForce) * m_deltaTimeSeconds
        );

        // Update pose (position and orientation) 
        m_pose = new Pose3d(
            m_pose.getTranslation().plus(m_velocity.times(m_deltaTimeSeconds)),
            new Rotation3d(
                m_pose.getRotation().getX() + m_rotation.getX() * m_deltaTimeSeconds,
                m_pose.getRotation().getY() + m_rotation.getY() * m_deltaTimeSeconds,
                m_pose.getRotation().getZ() + m_rotation.getZ() * m_deltaTimeSeconds
            )
        );

        // Publish the updated pose to AdvantageScope 
        m_posePublisher.set(m_pose);
        m_poseArrayPublisher.set(new Pose3d[] { m_pose });
        
        // Update simulation telemetry
        SmartDashboard.putString("Pose", m_pose.toString());
        SmartDashboard.putString("Velocity", m_velocity.toString());
        SmartDashboard.putString("Rotation", m_rotation.toString());
        
        // Update simulated gyro
        if (m_simulatedGyroAngle != null) {
            double currentAngle = m_simulatedGyroAngle.get();
            // Convert radians to degrees
            double rotationInDegrees = Units.radiansToDegrees(m_rotation.getZ() * m_deltaTimeSeconds);
            double newAngle = currentAngle + rotationInDegrees;
            m_simulatedGyroAngle.set(newAngle);
        }
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (shouldSimulate()) {
            updateSimulation();
        }
    }
}