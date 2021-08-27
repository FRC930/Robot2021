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
 * <h4>ShooterLEDs</h4> 
 * LED command for when the robot is shooting.
 */
public class ShooterLEDs extends CommandBase {

  // ----- CONSTANT(S) -----\\

  // The LED Subsystem (strip) itself
  private final LEDSubsystem m_LEDSubsystem;

  // LED length for the buffer
  private final int BUFFER_LENGTH;

  // ----- VARIABLE(S) -----\\

  /**
   * LED patterns are made on the buffer then sent to the LED strip via
   * LEDSubsystem
   */
  private AddressableLEDBuffer buffer;

  // Delay variable between each pattern change
  private int counter;

  // Indices for managing LED colors between blue and green
  private int blueIndex;
  private int greenIndex;

  /**
   * <h4>ShooterLEDs</h4> 
   * Creates an LED command for when the robot is shooting.
   * 
   * @param LEDSubsystem
   */
  public ShooterLEDs(LEDSubsystem LEDSubsystem) {

    m_LEDSubsystem = LEDSubsystem;
    BUFFER_LENGTH = m_LEDSubsystem.getBufferLength();
    buffer = new AddressableLEDBuffer(60);
    counter = 0;
    blueIndex = 0;
    greenIndex = 19;

    addRequirements(LEDSubsystem);

  } // End of constructor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    for (int i = 0; i < BUFFER_LENGTH; i++) {
      buffer.setRGB(i, 0, 0, 255);
    }
    m_LEDSubsystem.setBuffer(buffer);

  } // End of initialize()

  /**
   * <h4>moveLEDs</h4> Private helper method to move the light placement down the
   * strip.
   */
  private void moveLEDs() {

    // Increments all index positions
    blueIndex++;
    greenIndex++;

    // If said index variable is greater than or equal to BUFFER_LENGTH, set it to 0
    if (greenIndex >= BUFFER_LENGTH) {
      greenIndex = 0;
    }
    if (blueIndex >= BUFFER_LENGTH) {
      blueIndex = 0;
    }

  } // End of moveLEDs()

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /**
     * GENERAL SUMMARY
     * 
     * Everytime the counter reaches 3, the LEDs are given an updated pattern.
     * The pattern is just a rotating wheel of blue and green.
     */

    counter++;

    // Keeping track of animation speed.
    if (counter >= 3) {
      buffer.setRGB(blueIndex, 0, 0, 255); // Sets LED at blueIndex to blue
      moveLEDs(); // Moves the LEDs
      buffer.setRGB(greenIndex, 0, 255, 0); // Sets LED at greenIndex to green

      counter = 0;
    }
    
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
