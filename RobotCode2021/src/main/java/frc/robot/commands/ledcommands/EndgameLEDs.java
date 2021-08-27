/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ledcommands;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h4>EndgameLEDs</h4> 
 * LED command for when the robot is in endgame.
 */
public class EndgameLEDs extends CommandBase {

  //----- CONSTANT(S) -----\\

  // The LED Subsystem (strip) itself
  private final LEDSubsystem m_LEDSubsystem;

  // LED length for the buffer
  private final int BUFFER_LENGTH;

  //----- VARIABLE(S) -----\\

  // Alliance Color of the Robot
  private Alliance allianceColor;
  /**
   * LED patterns are made on the buffer then sent to the LED strip via
   * LEDSubsystem
   */
  private AddressableLEDBuffer buffer;

  // Delay variable between each pattern change
  private int counter;

  // Used to turn off an LED
  private int offIndex;
  // Used to turn on an LED
  private int onIndex;

  // ----- CONSTRUCTOR(S) -----\\

  /**
   * <h4>EndgameLEDs</h4> Creates a new LED command for when the robot is in
   * endgame.
   * 
   * @param LEDSubsystem Manages LEDs
   */
  public EndgameLEDs(LEDSubsystem LEDSubsystem) {

    m_LEDSubsystem = LEDSubsystem;
    BUFFER_LENGTH = m_LEDSubsystem.getBufferLength();
    buffer = new AddressableLEDBuffer(BUFFER_LENGTH);
    counter = 0;
    offIndex = 0;
    onIndex = 19;

    // Reserving the ledSubsystem for use by this command and this command only.
    addRequirements(LEDSubsystem);

  } // End of constructor

  // ----- METHOD(S) -----\\

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceColor = DriverStation.getInstance().getAlliance();

    for (int i = 0; i < BUFFER_LENGTH; i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
    m_LEDSubsystem.setBuffer(buffer);
  }

  /**
   * <h4>moveLEDs</h4> Private helper method to move the light placement down the
   * strip.
   */
  private void moveLEDs() {

    // Increments all index positions
    offIndex++;
    onIndex++;

    // If said index variable is greater than or equal to BUFFER_LENGTH, set it to 0
    if (offIndex >= BUFFER_LENGTH) {
      offIndex = 0;
    }
    if (onIndex >= BUFFER_LENGTH) {
      onIndex = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /**
     * GENERAL SUMMARY
     * 
     * Everytime counter reaches 3, the pattern is moved down one LED according to
     * the robot's alliance color.
     */

    counter++;

    // If counter is equal or greater to 3 (basically acting as a delay)
    if (counter >= 3) {

      // BLUE ALLIANCE
      if (allianceColor == Alliance.Blue) {
        buffer.setRGB(offIndex, 0, 0, 0); // Turns off an LED
        moveLEDs(); // Moves the LEDs
        buffer.setRGB(onIndex, 0, 0, 255); // Sets an LED to bright blue
      }

      // RED ALLIANCE
      else if (allianceColor == Alliance.Red) {
        buffer.setRGB(offIndex, 0, 0, 0); // Turns off an LED
        moveLEDs(); // Moves the LEDs
        buffer.setRGB(onIndex, 255, 0, 0); // Sets an LED to bright red
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
