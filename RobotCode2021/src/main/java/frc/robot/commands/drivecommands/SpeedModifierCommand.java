package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SpeedModifierCommand extends CommandBase{

    private final DriveSubsystem sdSubsystem;
    private double speedModifier;
    
    public SpeedModifierCommand(DriveSubsystem _sdSubsystem, double _speedModifier) {
        sdSubsystem = _sdSubsystem;
        speedModifier = _speedModifier;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        //System.out.println("SpeedModifierCommand reports: " + speedModifier);
        //sdSubsystem.setSpeedModifier(speedModifier);        
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
