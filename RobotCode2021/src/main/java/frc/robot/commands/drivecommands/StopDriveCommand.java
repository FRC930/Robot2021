/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class StopDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    public StopDriveCommand(DriveSubsystem dSubsystem){
        driveSubsystem = dSubsystem;
    }
    //ToDo: make generic for both tank and swerve
    @Override
    public void initialize() {  
        driveSubsystem.swerveStop(); 
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        
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
