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
 * <h4>IntakeLEDs</h4>
 * LED command for when the robot is intaking.
 */
public class IntakeLEDs extends CommandBase {

    // ----- CONSTANT(S) -----\\

    // The LED Subsystem (strip) itself
    private final LEDSubsystem m_LEDSubsystem;

    // LED length for the buffer
    private final int BUFFER_LENGTH;

    //----- VARIABLE(S) -----\\

    /**
     * LED patterns are made on the buffer then sent to the LED strip via
     * LEDSubsystem
     */
    private AddressableLEDBuffer buffer;

    // Delay variable between each pattern change
    private int counter;

    // Indices for managing color and when one should be on and off
    private int blueOffIndex;
    private int blueOnIndex;
    private int yellowOffIndex;
    private int yellowOnIndex;

    /**
     * <h4>IntakeLEDs</h4>
     * Creates a new LED command for when the robot is intaking.
     *
     * @param ledSubsystem Manages LEDs
     */
    public IntakeLEDs(LEDSubsystem LEDSubsystem) {
        
        m_LEDSubsystem = LEDSubsystem;
        BUFFER_LENGTH = m_LEDSubsystem.getBufferLength(); // gets the buffer length from the subsystem
        buffer = new AddressableLEDBuffer(BUFFER_LENGTH);
        counter = 0;
        blueOffIndex = 0;
        blueOnIndex = 19;
        yellowOffIndex = 30;
        yellowOnIndex = 49;

        // Reserving the ledSubsystem for use by this command and this command only.
        addRequirements(LEDSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Turns off all LEDs and sends the new LED states to the LEDSubsystem
        for (int i = 0; i < BUFFER_LENGTH; i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        m_LEDSubsystem.setBuffer(buffer);

    } // End of initialize()

    /**
     * <h4>moveLEDs</h4> Private helper method to move the light placement down the
     * strip.
     */
    private void moveLEDs() {

        // Increments all index positions
        blueOffIndex++;
        blueOnIndex++;
        yellowOffIndex++;
        yellowOnIndex++;

        // If said index variable is greater than or equal to BUFFER_LENGTH, set it to 0
        if (blueOnIndex >= BUFFER_LENGTH) {
            blueOnIndex = 0;
        }
        if (blueOffIndex >= BUFFER_LENGTH) {
            blueOffIndex = 0;
        }
        if (yellowOffIndex >= BUFFER_LENGTH) {
            yellowOffIndex = 0;
        }
        if (yellowOnIndex >= BUFFER_LENGTH) {
            yellowOnIndex = 0;
        }

    } // End of moveLEDs()

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        counter++;

        // If counter is equal or greater to 2 (basically acting as a delay)
        if (counter >= 2) {

            buffer.setRGB(blueOffIndex, 0, 0, 0);
            buffer.setRGB(yellowOffIndex, 0, 0, 0);
            moveLEDs(); // Moves the LEDs
            buffer.setRGB(blueOnIndex, 0, 0, 255); // Blue
            buffer.setRGB(yellowOnIndex, 255, 255, 0); // Yellow

            // Resets the counter to 0 once the pattern is set
            counter = 0;
        }

        m_LEDSubsystem.setBuffer(buffer);

    } // End of Execute()

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
