package frc.robot.commands.endgamecommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgameSubsystem;
import frc.robot.utilities.ShuffleboardUtility;

/**
 * controls the two hooks for the endgame
 */
public class EndgameRunCommand extends CommandBase{
    
    private EndgameSubsystem endgameSubsystem;
    ShuffleboardUtility shuffleboardUtility;
    
    //Constructor

    /**
     * this constructs the class
     * @param _endgameSubsystem controls endgame motor
     */
    public EndgameRunCommand(EndgameSubsystem _endgameSubsystem) {
        shuffleboardUtility = ShuffleboardUtility.getInstance();
        endgameSubsystem = _endgameSubsystem;
        addRequirements(endgameSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
       //set speed of motor
        endgameSubsystem.setSpeed(0.8);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        endgameSubsystem.setSpeed(0.0);
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
