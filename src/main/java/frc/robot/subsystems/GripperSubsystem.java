// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlueRoboticsNewton;
import frc.robot.util.Elastic;

public class GripperSubsystem extends SubsystemBase {
    // Newton gripper
    private final BlueRoboticsNewton m_newtonGripper = new BlueRoboticsNewton(9);
    
    // Timer for gripper control
    private final Timer m_gripperTimer = new Timer();
    
    // For notifications
    private String m_lastGripperState = "Stopped";
    
    public GripperSubsystem() {
        SendableRegistry.addChild(this, m_newtonGripper);
        SendableRegistry.addLW(m_newtonGripper, "NewtonGripper");
        
        m_gripperTimer.reset();
        m_gripperTimer.start();
    }
    
    @Override
    public void periodic() {
        // Check if we need to stop the gripper after the time limit
        if (m_gripperTimer.get() >= 4.0 && !m_lastGripperState.equals("Stopped")) {
            stop();
            m_gripperTimer.stop();
        }
    }
    
    /**
     * Open the gripper.
     */
    public void open() {
        m_newtonGripper.set(1.0); // Full forward power (open)
        updateState("Opening");
        m_gripperTimer.reset();
        m_gripperTimer.start();
    }
    
    /**
     * Close the gripper.
     */
    public void close() {
        m_newtonGripper.set(-1.0); // Full reverse power (close)
        updateState("Closing");
        m_gripperTimer.reset();
        m_gripperTimer.start();
    }
    
    /**
     * Stop the gripper.
     */
    public void stop() {
        m_newtonGripper.set(0.0);
        updateState("Stopped");
    }
    
    /**
     * Update the gripper state and send notification if changed.
     *
     * @param newState The new state of the gripper
     */
    private void updateState(String newState) {
        if (!newState.equals(m_lastGripperState)) {
            Elastic.sendNotification(
                new Elastic.Notification()
                    .withLevel(Elastic.Notification.NotificationLevel.INFO)
                    .withTitle("Gripper " + newState)
                    .withDescription("Power set to: " + m_newtonGripper.getVoltage())
                    .withDisplaySeconds(5.0)
            );
            
            m_lastGripperState = newState;
            SmartDashboard.putString("Gripper Status", newState);
        }
    }
    
    /**
     * Get the current state of the gripper.
     *
     * @return Current gripper state as a string
     */
    public String getState() {
        return m_lastGripperState;
    }
    
    /**
     * Run the gripper at 50% for testing.
     */
    public void testGripper() {
        m_newtonGripper.set(0.5);
        updateState("Testing");
    }
}