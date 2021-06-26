package frc.robot.commands.endgamecommands;

import java.util.logging.Logger;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EndgameSubsystem;

public class EndgameRunCommand extends CommandBase{
    
    private EndgameSubsystem endgameSubsystem;
    private double speed;

    //LIMITS CHANGE
    private final double highLimit = 1000;
    private final double lowLimit = 0;
    

    public EndgameRunCommand(EndgameSubsystem _endgameSubsystem) {
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
        endgameSubsystem.setSpeed(0.8);
        /*if(!endgameSubsystem.getLimitState()){
            if(endgameSubsystem.getUpState()){
                if(endgameSubsystem.getRawEncoderPosition() == highLimit){
                    endgameSubsystem.setSpeed(0);
                    endgameSubsystem.setLimitState(true);
                }
            }
            else{
                if(endgameSubsystem.getRawEncoderPosition() == lowLimit){
                    endgameSubsystem.setSpeed(0);
                    endgameSubsystem.setLimitState(true);
                }
            }
        }*/
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
