package frc.robot.commands.ultrasoniccommands;

import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem.GALACTIC_PATH;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UltrasonicPingCommand extends CommandBase {
    private UltrasonicSubsystem ultrasonicSubsystem;

    public UltrasonicPingCommand(UltrasonicSubsystem _ultrasonicSubsystem) {
        ultrasonicSubsystem = _ultrasonicSubsystem;

        addRequirements(_ultrasonicSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        System.out.println("Dist;" + ultrasonicSubsystem.getDistance());

        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
