// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlueRoboticsBasicESC;

public class DriveSubsystem extends SubsystemBase {
    // Thruster definitions
    private final BlueRoboticsBasicESC m_leftFront45 = new BlueRoboticsBasicESC(0);
    private final BlueRoboticsBasicESC m_leftRear45 = new BlueRoboticsBasicESC(1);
    private final BlueRoboticsBasicESC m_rightFront45 = new BlueRoboticsBasicESC(2);
    private final BlueRoboticsBasicESC m_rightRear45 = new BlueRoboticsBasicESC(3);
    private final BlueRoboticsBasicESC m_leftFrontForward = new BlueRoboticsBasicESC(4);
    private final BlueRoboticsBasicESC m_leftRearForward = new BlueRoboticsBasicESC(5);
    private final BlueRoboticsBasicESC m_rightFrontForward = new BlueRoboticsBasicESC(6);
    private final BlueRoboticsBasicESC m_rightRearForward = new BlueRoboticsBasicESC(7);
    
    // Gyro definition
    private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
    
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