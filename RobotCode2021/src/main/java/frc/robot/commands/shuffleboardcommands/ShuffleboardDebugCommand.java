package frc.robot.commands.shuffleboardcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.ShuffleboardUtility;

public class ShuffleboardDebugCommand extends CommandBase{
    
    //-------- DELCARATIONS --------\\

    private ShuffleboardUtility shuffleboard = ShuffleboardUtility.getInstance();

    //-------- CONSTRUCTOR --------\\

    public ShuffleboardDebugCommand() {

        // hehe, funny code line :))

        //addRequirements();
    }

    //-------- COMMANDBASE METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // turn all access off to ensure that only the following are toggled on
        shuffleboard.allAccessFalse();

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
