package frc.robot.commands.endgamecommands;

import java.util.logging.Logger;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EndgameSubsystem;

public class EndgameCommand extends CommandBase{
    
    private EndgameSubsystem endgameSubsystem;
    private double speed;

    public EndgameCommand(EndgameSubsystem _endgameSubsystem, double _speed) {
        endgameSubsystem = _endgameSubsystem;
        speed = _speed;

        addRequirements(endgameSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   
        //endgameSubsystem.setSpeed(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        // TODO: add code here
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
