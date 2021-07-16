package frc.robot.commands.endgamecommands;

import java.util.logging.Logger;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EndgameSubsystem;
import frc.robot.utilities.ShuffleboardUtility;

public class EndgameTestCommand extends CommandBase{
    
    private EndgameSubsystem endgameSubsystem;
    ShuffleboardUtility shuffleboardUtility;
    double speed;
    
    //Constructor
    public EndgameTestCommand(EndgameSubsystem _endgameSubsystem, double _speed) {

        shuffleboardUtility = ShuffleboardUtility.getInstance();
        endgameSubsystem = _endgameSubsystem;
        speed =_speed;
        addRequirements(endgameSubsystem);
    }

    //-------- METHODS --------\\


    // IMPORTANT!! not used as a command, only for test mode
    @Override
    public void execute() { 
       //set speed of motor
        endgameSubsystem.setSpeed(speed);
    }
}
