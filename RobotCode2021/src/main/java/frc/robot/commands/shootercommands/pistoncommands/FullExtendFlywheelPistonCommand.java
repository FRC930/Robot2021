/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.shootercommands.pistoncommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem.SolenoidValues;
import frc.robot.utilities.ShuffleboardUtility;

/**
 * this class fully extends the piston of the shooter
 */
public class FullExtendFlywheelPistonCommand extends CommandBase {

  private FlywheelPistonSubsystem flywheelAngleSubsystem;
  private ShuffleboardUtility shuffleboardUtility;

    //-------- CONSTRUCTOR --------\\
  /**
   * this constructs the class
   * @param flywheelAngleSubsystem controls the piston to the shooter
   */
    public FullExtendFlywheelPistonCommand(FlywheelPistonSubsystem flywheelAngleSubsystem) {
    this.flywheelAngleSubsystem = flywheelAngleSubsystem;
    shuffleboardUtility = ShuffleboardUtility.getInstance();
    addRequirements(flywheelAngleSubsystem);
  }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelAngleSubsystem.setBottom(SolenoidValues.EXTEND);
    shuffleboardUtility.putShooterAngle(true);
  }  
  
//-------- COMMANDBASE METHODS --------\\
  
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
     return false;
   }

} // end of class ExtendFlywheelPistonCommand