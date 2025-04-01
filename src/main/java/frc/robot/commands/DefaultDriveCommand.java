// Copyright (c) Ada Tessar (me@adabit.org)
package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SimulationSubsystem;

/**
 * Default drive command that handles robot movement in teleop mode.
 * Uses supplied functions to continuously get control inputs from controllers.
 */
public class DefaultDriveCommand extends Command {
    private final DriveSubsystem m_drive;
    private final SimulationSubsystem m_simulation;
    private final DoubleSupplier m_forwardSupplier;
    private final DoubleSupplier m_strafeSupplier;
    private final DoubleSupplier m_verticalSupplier;
    private final DoubleSupplier m_yawSupplier;
    private final DoubleSupplier m_rollSupplier;
    private final DoubleSupplier m_pitchSupplier;
    private final BooleanSupplier m_poolRelativeSupplier;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param drive               The drive subsystem to use
     * @param simulation          The simulation subsystem to use
     * @param forwardSupplier     Supplier for forward/backward movement
     * @param strafeSupplier      Supplier for left/right movement
     * @param verticalSupplier    Supplier for up/down movement
     * @param yawSupplier         Supplier for yaw rotation
     * @param rollSupplier        Supplier for roll control
     * @param pitchSupplier       Supplier for pitch control
     * @param poolRelativeSupplier Supplier for pool-relative toggle state
     */
    public DefaultDriveCommand(
            DriveSubsystem drive,
            SimulationSubsystem simulation,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier verticalSupplier,
            DoubleSupplier yawSupplier,
            DoubleSupplier rollSupplier,
            DoubleSupplier pitchSupplier,
            BooleanSupplier poolRelativeSupplier) {
        m_drive = drive;
        m_simulation = simulation;
        m_forwardSupplier = forwardSupplier;
        m_strafeSupplier = strafeSupplier;
        m_verticalSupplier = verticalSupplier;
        m_yawSupplier = yawSupplier;
        m_rollSupplier = rollSupplier;
        m_pitchSupplier = pitchSupplier;
        m_poolRelativeSupplier = poolRelativeSupplier;
        
        // Add requirements
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Read the control inputs
        double forward = m_forwardSupplier.getAsDouble();
        double strafe = m_strafeSupplier.getAsDouble();
        double vertical = m_verticalSupplier.getAsDouble();
        double yaw = m_yawSupplier.getAsDouble();
        double roll = m_rollSupplier.getAsDouble();
        double pitch = m_pitchSupplier.getAsDouble();
        boolean poolRelative = m_poolRelativeSupplier.getAsBoolean();
        
        // Transform coordinates for pool-relative control if needed
        double poolX = forward;
        double poolY = strafe;
        double poolZ = vertical;
        
        if (poolRelative) {
            double[] transformed = m_drive.calculatePoolRelative(forward, strafe, vertical);
            poolX = transformed[0];
            poolY = transformed[1];
            poolZ = transformed[2];
        }
        
        // Drive the robot
        m_drive.drive(poolX, poolY, poolZ, yaw, pitch, roll);
        
        // Update simulation
        m_simulation.setInputs(poolX, poolY, poolZ, yaw, pitch);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_drive.stopAll();
    }

    @Override
    public boolean isFinished() {
        // Default command should never finish on its own
        return false;
    }
}