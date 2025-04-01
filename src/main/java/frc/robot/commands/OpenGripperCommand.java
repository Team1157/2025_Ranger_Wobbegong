// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;

/**
 * Command to open the Newton gripper.
 */
public class OpenGripperCommand extends Command {
    private final GripperSubsystem m_gripper;
    
    /**
     * Creates a new OpenGripperCommand.
     *
     * @param gripper The gripper subsystem to use
     */
    public OpenGripperCommand(GripperSubsystem gripper) {
        m_gripper = gripper;
        addRequirements(m_gripper);
    }
    
    @Override
    public void initialize() {
        m_gripper.open();
    }
    
    @Override
    public boolean isFinished() {
        // This is an "instant" command that triggers the gripper to open
        // and then finishes immediately - the gripper subsystem will handle
        // the timing to stop after 4 seconds
        return true;
    }
}