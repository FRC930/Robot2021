/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ledcommands;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h4>IdleLEDs</h4>
 * LED command for when the robot is in the idle state.
 */
public class IdleLEDs extends CommandBase {
  
  //----- CONSTANT(S) -----\\

  // The LED Subsystem (strip) itself
  private final LEDSubsystem m_LEDSubsystem;

  // LED length for the buffer
  private final int BUFFER_LENGTH;

  //----- VARIABLE(S) -----\\

  /** LED patterns are made on the buffer then sent to the LED strip via LEDSubsystem */
  private AddressableLEDBuffer buffer;

  // Delay variable between each pattern change
  private int counter;
  // Used to switch between patterns
  private boolean animCheck;

  //----- CONSTRUCTOR(S) -----\\

  /**
   * <h4>IdleLEDs</h4>
   * Creates a new LED command for when the robot is in the idle state.
   *
   * @param ledSubsystem Manages LEDs
   */
  public IdleLEDs(LEDSubsystem ledSubsystem) {

    m_LEDSubsystem = ledSubsystem;
    BUFFER_LENGTH = m_LEDSubsystem.getBufferLength();
    buffer = new AddressableLEDBuffer(BUFFER_LENGTH); // Creates a new buffer object
    counter = 0;
    animCheck = false;

    // Reserving the ledSubsystem for use by this command and this command only.
    addRequirements(ledSubsystem);

  } // End of constructor

  //----- METHOD(S) -----\\

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  } // End of initialize()

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /**
     * GENERAL SUMMARY
     * 
     * Everytime counter reaches 32, the patterns are switched.
     * There are two patterns:
     *      10101010
     *      01010101
     */

    counter++;

    // If counter is equal or greater to 32 (basically acting as a delay)
    if (counter >= 32) {

      if (animCheck == true) {

        /**
         * sets the LED pattern to:
         * 10101010101010......
         * 
         * (0 being off, 1 being yellow)
         */
        for (int i = 0; i < BUFFER_LENGTH; i += 2) {
          buffer.setRGB(i, 255, 255, 0); // Yellow
        }

        // Sets animCheck to false for pattern switch next time the delay is finished
        animCheck = false;

      } else {

        /**
         * sets the LED pattern to:
         * 01010101010101......
         * 
         * (0 being off, 1 being yellow)
         */
        for (int i = 1; i < BUFFER_LENGTH; i += 2) {
          buffer.setRGB(i, 255, 255, 0); // Yellow
        }

        // Sets animCheck to true for pattern switch next time the delay is finished
        animCheck = true;
      }

      // Resets the counter to 0 once the pattern is set
      counter = 0;
    }

    // The pattern is sent to the LED strip to be displayed
    m_LEDSubsystem.setBuffer(buffer);

  } // End of execute()

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  } // End of end()

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } // End of isFinished()
}
