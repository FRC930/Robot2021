package frc.robot.commands.endgamecommands;

import java.util.logging.Logger;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EndgameSubsystem;

public class EndgameCommandFlipState extends CommandBase{
    
    private EndgameSubsystem endgameSubsystem;

    public EndgameCommandFlipState(EndgameSubsystem _endgameSubsystem) {
        endgameSubsystem = _endgameSubsystem;
        addRequirements(endgameSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   
        endgameSubsystem.setLimitState(false);
        endgameSubsystem.setUpState(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        endgameSubsystem.setLimitState(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: add code here
        return true;
    }

}
