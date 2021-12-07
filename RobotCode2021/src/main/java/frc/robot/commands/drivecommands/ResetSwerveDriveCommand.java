/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.NewDriveSubsystem;

/**
 * <h3>ResetSwerveDriveCommand</h3>
 * 
 * This class is used for resetting the wheel angles to zero and zeroing the
 * gyro. Used mainly for testing.
 */
public class ResetSwerveDriveCommand extends CommandBase {
    private NewDriveSubsystem driveSubsystem;

    /**
     * <h3>ResetSwerveDriveCommand</h3>
     * 
     * Takes a {@link frc.robot.subsystems.DriveSubsystem DriveSubsystem} and zeros
     * the wheels and gyro whenever it is executed
     * 
     * @param dSubsystem the instance of {@link frc.robot.subsystems.DriveSubsystem
     *                   DriveSubsystem} to use
     */
    public ResetSwerveDriveCommand(NewDriveSubsystem dSubsystem) {
        driveSubsystem = dSubsystem;
        addRequirements(dSubsystem);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // driveSubsystem.swerveResetWheels();
        driveSubsystem.rebootGyro();
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
