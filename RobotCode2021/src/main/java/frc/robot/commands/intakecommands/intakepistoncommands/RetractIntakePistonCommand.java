/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.intakecommands.intakepistoncommands;

import java.util.logging.Logger;

import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;

//-------- COMMAND CLASS --------\\
/**
 * <h4>ExtendIntakePistonCommand</h4> 
 * Retracts pistons on the intake.
 */
public class RetractIntakePistonCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  private static final Logger logger = Logger.getLogger(RetractIntakePistonCommand.class.getName());

  // -------- DECLARATIONS --------\\

  private final IntakePistonSubsystem intakePistonSubsystem;
  private ShuffleboardUtility shuffleboardUtility;

  // -------- CONSTRUCTOR --------\\
  /**
   * 
   * @param iPistons instantiated {@link IntakePistonSubsystem} object.
   */
  public RetractIntakePistonCommand(IntakePistonSubsystem iPiston) {
    intakePistonSubsystem = iPiston;
    shuffleboardUtility = ShuffleboardUtility.getInstance();

    addRequirements(iPiston); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- COMMANDBASE METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    // Sets pistons to up state
    intakePistonSubsystem.setIntakePistonState(intakePistonSubsystem.getIntakePistonUpState());

    shuffleboardUtility.putIntakeIndicator(intakePistonSubsystem.getIntakePistonState());
    logger.log(Constants.LOG_LEVEL_FINE, "Retracting the intake pistons (command)...");
  }

  @Override // Returns true when the command should end.
  public boolean isFinished() {
    return true;
  }

} // End of class RetractIntakePistonCommand
