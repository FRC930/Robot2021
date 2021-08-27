// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// //-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h4>LEDSubsystem</h4>
 * Subsystem class for managing the LED hardware
 */
public class LEDSubsystem extends SubsystemBase {

    //----- CONSTANT(S) -----\\

    private final int BUFFER_LENGTH;

    //----- VARIABLE(S) -----\\

    private AddressableLED leds;

    //----- CONSTRUCTOR(S) -----\\

    /**
     * <h4>LEDSubsystem</h4>
     * Creates a subsystem class for managing the LED hardware
     * 
     * @param port LED Strip Port
     * @param length Length of the LED Strip
     */
    public LEDSubsystem(int port, int length) {
        BUFFER_LENGTH = length;
        leds = new AddressableLED(port); //initialization of the AdressableLED
        leds.setLength(length); //Sets the LED Strip length once
    }

    //----- METHOD(S) -----\\
    
    /**
     * <h4>start</h4>
     * Call the leds to start the subsystem
     */
    public void start(){
        leds.start();
    } // End of start()

    /**
     * <h4>setBuffer</h4>
     * Sets the buffer(color strips) for the LED Strip
     * 
     * @param buffer
     */
    public void setBuffer(AddressableLEDBuffer buffer){

        leds.setData(buffer);

    } // End of setBuffer()

    /**
     * <h4>getBufferLength</h4>
     * Returns the buffer length
     * 
     * @return BUFFER_LENGTH
     */
    public int getBufferLength() {

        return BUFFER_LENGTH;

    } // End of getBufferLength()
    
} // end of class LEDSubsystem