package frc.robot.commands.shuffleboardcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.ShuffleboardUtility;

/**
 * <h4>ShufflboardDebugCommand</h4> Sets the access state for all data to true
 * for debugging purposes.
 */
public class ShuffleboardDebugCommand extends CommandBase {

    // -------- DELCARATIONS --------\\
    // Gets utility instance from singleton
    private ShuffleboardUtility shuffleboard = ShuffleboardUtility.getInstance();

    // -------- CONSTRUCTOR --------\\

    public ShuffleboardDebugCommand() {
    }

    // -------- COMMANDBASE METHODS --------\\

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Sets all groups to false to align them
        shuffleboard.allAccessFalse();
        // Sets all groups to true
        shuffleboard.toggleEndgameAccess();
        shuffleboard.toggleFlywheelAccess();
        shuffleboard.toggleHopperAccess();
        shuffleboard.toggleIntakeAccess();
        shuffleboard.toggleLimelightAccess();
        shuffleboard.toggleMiscellaneousAccess();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
