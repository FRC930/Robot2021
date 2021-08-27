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
 * fully retracts the pistons on the shooter
 */
public class FullRetractFlywheelPistonCommand extends CommandBase {

  //-------- DECLARATIONS --------\\

  private FlywheelPistonSubsystem flywheelPistonSubsystem;
  private ShuffleboardUtility shuffleboardUtility;

  //-------- CONSTRUCTOR --------\\
    /**
     * this constructs the class
     * @param flywheelPistonSubsystem controls the pistons for the shooter
     */
  public FullRetractFlywheelPistonCommand(FlywheelPistonSubsystem flywheelPistonSubsystem){
    this.flywheelPistonSubsystem = flywheelPistonSubsystem;
    shuffleboardUtility = ShuffleboardUtility.getInstance();
    addRequirements(flywheelPistonSubsystem);
  }  
  
  //-------- COMMANDBASE METHODS --------\\

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // flywheelPistonSubsystem.setTop(SolenoidValues.RETRACT);
    flywheelPistonSubsystem.setBottom(SolenoidValues.RETRACT);
    shuffleboardUtility.putShooterAngle(false);
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
    return false;
  }

} // end of class RetractFlywheelPistonCommand