// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlueRoboticsBasicESC;

public class DriveSubsystem extends SubsystemBase {
    // Thruster definitions
    private final BlueRoboticsBasicESC m_leftFront45 = new BlueRoboticsBasicESC(0);
    private final BlueRoboticsBasicESC m_leftRear45 = new BlueRoboticsBasicESC(5);
    private final BlueRoboticsBasicESC m_rightFront45 = new BlueRoboticsBasicESC(7);
    private final BlueRoboticsBasicESC m_rightRear45 = new BlueRoboticsBasicESC(1);
    private final BlueRoboticsBasicESC m_leftFrontForward = new BlueRoboticsBasicESC(6);
    private final BlueRoboticsBasicESC m_leftRearForward = new BlueRoboticsBasicESC(4);
    private final BlueRoboticsBasicESC m_rightFrontForward = new BlueRoboticsBasicESC(3);
    private final BlueRoboticsBasicESC m_rightRearForward = new BlueRoboticsBasicESC(2);
    
    // Gyro definition
    private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
    
    // NetworkTable publishers for AdvantageScope
    private DoubleArrayPublisher m_thrusterOutputsPublisher;
    private DoubleArrayPublisher m_gyroDataPublisher;
    private DoubleArrayPublisher m_accelerometerDataPublisher;
    private DoublePublisher m_temperaturePublisher;
    private DoublePublisher m_pressurePublisher;
    
    // Current drive values for logging
    private double m_currentX = 0.0;
    private double m_currentY = 0.0;
    private double m_currentZ = 0.0;
    private double m_currentYaw = 0.0;
    private double m_currentPitch = 0.0;
    private double m_currentRoll = 0.0;
    
    public DriveSubsystem() {
        // Set up thrusters in the SendableRegistry for debugging
        SendableRegistry.addChild(this, m_leftFront45);
        SendableRegistry.addLW(m_leftFront45, "LeftFront45");
        
        SendableRegistry.addChild(this, m_leftRear45);
        SendableRegistry.addLW(m_leftRear45, "LeftRear45");
        
        SendableRegistry.addChild(this, m_rightFront45);
        SendableRegistry.addLW(m_rightFront45, "RightFront45");
        
        SendableRegistry.addChild(this, m_rightRear45);
        SendableRegistry.addLW(m_rightRear45, "RightRear45");
        
        SendableRegistry.addChild(this, m_leftFrontForward);
        SendableRegistry.addLW(m_leftFrontForward, "LeftFrontForward");
        
        SendableRegistry.addChild(this, m_leftRearForward);
        SendableRegistry.addLW(m_leftRearForward, "LeftRearForward");
        
        SendableRegistry.addChild(this, m_rightFrontForward);
        SendableRegistry.addLW(m_rightFrontForward, "RightFrontForward");
        
        SendableRegistry.addChild(this, m_rightRearForward);
        SendableRegistry.addLW(m_rightRearForward, "RightRearForward");
        
        // Initialize NetworkTable publishers
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        m_thrusterOutputsPublisher = nt.getDoubleArrayTopic("/Drive/ThrusterOutputs").publish();
        m_gyroDataPublisher = nt.getDoubleArrayTopic("/Drive/GyroData").publish();
        m_accelerometerDataPublisher = nt.getDoubleArrayTopic("/Drive/AccelerometerData").publish();
        m_temperaturePublisher = nt.getDoubleTopic("/Drive/Temperature").publish();
        m_pressurePublisher = nt.getDoubleTopic("/Drive/Pressure").publish();
    }
    
    @Override
    public void periodic() {
        // Log thruster outputs to NetworkTables
        m_thrusterOutputsPublisher.set(new double[] {
            m_leftFront45.get(),
            m_leftRear45.get(),
            m_rightFront45.get(),
            m_rightRear45.get(),
            m_leftFrontForward.get(),
            m_leftRearForward.get(),
            m_rightFrontForward.get(),
            m_rightRearForward.get()
        });
        
        // Log IMU data
        m_gyroDataPublisher.set(new double[] {
            m_imu.getAngle(), // yaw
            m_imu.getGyroAngleY(), // pitch
            m_imu.getGyroAngleX(), // roll
            m_imu.getGyroRateZ(), // yaw rate
            m_imu.getGyroRateY(), // pitch rate
            m_imu.getGyroRateX()  // roll rate
        });
        
        m_accelerometerDataPublisher.set(new double[] {
            m_imu.getAccelX(),
            m_imu.getAccelY(),
            m_imu.getAccelZ()
        });
        
        m_temperaturePublisher.set(m_imu.getTemperature());
        m_pressurePublisher.set(m_imu.getBarometricPressure());
        
        // Log to AdvantageKit
        Logger.recordOutput("Drive/ThrusterOutputs/LeftFront45", m_leftFront45.get());
        Logger.recordOutput("Drive/ThrusterOutputs/LeftRear45", m_leftRear45.get());
        Logger.recordOutput("Drive/ThrusterOutputs/RightFront45", m_rightFront45.get());
        Logger.recordOutput("Drive/ThrusterOutputs/RightRear45", m_rightRear45.get());
        Logger.recordOutput("Drive/ThrusterOutputs/LeftFrontForward", m_leftFrontForward.get());
        Logger.recordOutput("Drive/ThrusterOutputs/LeftRearForward", m_leftRearForward.get());
        Logger.recordOutput("Drive/ThrusterOutputs/RightFrontForward", m_rightFrontForward.get());
        Logger.recordOutput("Drive/ThrusterOutputs/RightRearForward", m_rightRearForward.get());
        
        // Log IMU data to AdvantageKit
        Logger.recordOutput("Drive/IMU/Yaw", m_imu.getAngle());
        Logger.recordOutput("Drive/IMU/Pitch", m_imu.getGyroAngleY());
        Logger.recordOutput("Drive/IMU/Roll", m_imu.getGyroAngleX());
        Logger.recordOutput("Drive/IMU/YawRate", m_imu.getGyroRateZ());
        Logger.recordOutput("Drive/IMU/PitchRate", m_imu.getGyroRateY());
        Logger.recordOutput("Drive/IMU/RollRate", m_imu.getGyroRateX());
        Logger.recordOutput("Drive/IMU/AccelX", m_imu.getAccelX());
        Logger.recordOutput("Drive/IMU/AccelY", m_imu.getAccelY());
        Logger.recordOutput("Drive/IMU/AccelZ", m_imu.getAccelZ());
        Logger.recordOutput("Drive/IMU/Temperature", m_imu.getTemperature());
        Logger.recordOutput("Drive/IMU/Pressure", m_imu.getBarometricPressure());
        
        // Log current drive values
        Logger.recordOutput("Drive/Commands/X", m_currentX);
        Logger.recordOutput("Drive/Commands/Y", m_currentY);
        Logger.recordOutput("Drive/Commands/Z", m_currentZ);
        Logger.recordOutput("Drive/Commands/Yaw", m_currentYaw);
        Logger.recordOutput("Drive/Commands/Pitch", m_currentPitch);
        Logger.recordOutput("Drive/Commands/Roll", m_currentRoll);
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Yaw", m_imu.getAngle());
        SmartDashboard.putNumber("Pitch", m_imu.getGyroAngleY());
        SmartDashboard.putNumber("Roll", m_imu.getGyroAngleX());
    }
    
    /**
     * Drive the robot using raw values.
     *
     * @param poolX    Forward/backward movement
     * @param poolY    Left/right movement
     * @param poolZ    Up/down movement
     * @param yaw      Yaw rotation
     * @param pitch    Pitch rotation
     * @param roll     Roll rotation
     */
    public void drive(double poolX, double poolY, double poolZ, double yaw, double pitch, double roll) {
        // Store current values for logging
        m_currentX = poolX;
        m_currentY = poolY;
        m_currentZ = poolZ;
        m_currentYaw = yaw;
        m_currentPitch = pitch;
        m_currentRoll = roll;
        
        // Set power to thrusters for 3D movement
        m_leftFront45.set(poolY + poolX);
        m_leftRear45.set(-poolY + poolX);
        m_rightFront45.set(poolY - poolX);
        m_rightRear45.set(-poolY - poolX);

        // Vertical, pitch, roll, and yaw control
        m_leftFrontForward.set(poolZ + roll + yaw + pitch);
        m_leftRearForward.set(poolZ - roll + yaw - pitch);
        m_rightFrontForward.set(poolZ + roll - yaw + pitch);
        m_rightRearForward.set(poolZ - roll - yaw - pitch);
        
        // Log thruster powers to AdvantageKit as they're set
        Logger.recordOutput("Drive/CommandedPowers/LeftFront45", poolY + poolX);
        Logger.recordOutput("Drive/CommandedPowers/LeftRear45", -poolY + poolX);
        Logger.recordOutput("Drive/CommandedPowers/RightFront45", poolY - poolX);
        Logger.recordOutput("Drive/CommandedPowers/RightRear45", -poolY - poolX);
        Logger.recordOutput("Drive/CommandedPowers/LeftFrontForward", poolZ + roll + yaw + pitch);
        Logger.recordOutput("Drive/CommandedPowers/LeftRearForward", poolZ - roll + yaw - pitch);
        Logger.recordOutput("Drive/CommandedPowers/RightFrontForward", poolZ + roll - yaw + pitch);
        Logger.recordOutput("Drive/CommandedPowers/RightRearForward", poolZ - roll - yaw - pitch);
    }
    
    /**
     * Calculate pool-relative controls based on IMU data.
     *
     * @param robotX    Robot-relative forward/backward
     * @param robotY    Robot-relative left/right
     * @param robotZ    Robot-relative up/down
     * @return          Pool-relative translation
     */
    public double[] calculatePoolRelative(double robotX, double robotY, double robotZ) {
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
        double rotatedX = robotX * (cosYaw * cosPitch) + 
                          robotY * (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) + 
                          robotZ * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
                          
        double rotatedY = robotX * (sinYaw * cosPitch) + 
                          robotY * (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) + 
                          robotZ * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);
                          
        double rotatedZ = robotX * (-sinPitch) + 
                          robotY * (cosPitch * sinRoll) + 
                          robotZ * (cosPitch * cosRoll);
                          
        // Log transformation values
        Logger.recordOutput("Drive/PoolRelative/InputX", robotX);
        Logger.recordOutput("Drive/PoolRelative/InputY", robotY);
        Logger.recordOutput("Drive/PoolRelative/InputZ", robotZ);
        Logger.recordOutput("Drive/PoolRelative/OutputX", rotatedX);
        Logger.recordOutput("Drive/PoolRelative/OutputY", rotatedY);
        Logger.recordOutput("Drive/PoolRelative/OutputZ", rotatedZ);
        Logger.recordOutput("Drive/PoolRelative/TransformMatrix", new double[] {
            cosYaw * cosPitch, cosYaw * sinPitch * sinRoll - sinYaw * cosRoll, cosYaw * sinPitch * cosRoll + sinYaw * sinRoll,
            sinYaw * cosPitch, sinYaw * sinPitch * sinRoll + cosYaw * cosRoll, sinYaw * sinPitch * cosRoll - cosYaw * sinRoll,
            -sinPitch, cosPitch * sinRoll, cosPitch * cosRoll
        });

        return new double[] { rotatedX, rotatedY, rotatedZ };
    }
    
    /**
     * Get the current yaw angle from the IMU.
     *
     * @return Current yaw angle in degrees
     */
    public double getYaw() {
        return m_imu.getAngle();
    }
    
    /**
     * Get the current pitch angle from the IMU.
     *
     * @return Current pitch angle in degrees
     */
    public double getPitch() {
        return m_imu.getGyroAngleY();
    }
    
    /**
     * Get the current roll angle from the IMU.
     *
     * @return Current roll angle in degrees
     */
    public double getRoll() {
        return m_imu.getGyroAngleX();
    }
    
    /**
     * Get the current rotation from the IMU.
     *
     * @return Current rotation as a Rotation3d
     */
    public Rotation3d getRotation() {
        return new Rotation3d(
            Math.toRadians(m_imu.getGyroAngleX()),
            Math.toRadians(m_imu.getGyroAngleY()),
            Math.toRadians(m_imu.getAngle())
        );
    }
    
    /**
     * Calibrate the gyro.
     */
    public void calibrateGyro() {
        m_imu.calibrate();
    }
    
    /**
     * Reset the gyro to zero.
     */
    public void resetGyro() {
        m_imu.reset();
    }
    
    /**
     * Stop all thrusters.
     */
    public void stopAll() {
        m_leftFront45.set(0);
        m_leftRear45.set(0);
        m_rightFront45.set(0);
        m_rightRear45.set(0);
        m_leftFrontForward.set(0);
        m_leftRearForward.set(0);
        m_rightFrontForward.set(0);
        m_rightRearForward.set(0);
    }
    
    /**
     * Run all thrusters at 50% for testing.
     */
    public void testThrusters() {
        m_leftFront45.set(0.5);
        m_leftRear45.set(0.5);
        m_rightFront45.set(0.5);
        m_rightRear45.set(0.5);
        m_leftFrontForward.set(0.5);
        m_leftRearForward.set(0.5);
        m_rightFrontForward.set(0.5);
        m_rightRearForward.set(0.5);
    }
}