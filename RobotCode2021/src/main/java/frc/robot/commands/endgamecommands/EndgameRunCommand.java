package frc.robot.commands.endgamecommands;

import java.util.logging.Logger;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EndgameSubsystem;
import frc.robot.utilities.ShuffleboardUtility;

public class EndgameRunCommand extends CommandBase{
    
    private EndgameSubsystem endgameSubsystem;
    ShuffleboardUtility shuffleboardUtility;
    
    //Constructor
    public EndgameRunCommand(EndgameSubsystem _endgameSubsystem) {
        shuffleboardUtility = ShuffleboardUtility.getInstance();
        endgameSubsystem = _endgameSubsystem;
        addRequirements(endgameSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   
        //endgameSubsystem.setSpeed(0.8);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
       //set speed of motor
        endgameSubsystem.setSpeed(-0.8);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        endgameSubsystem.setSpeed(0.0);
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: add code here
        return false;
    }

}
