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


// //-------- SUBSYSTEM CLASS --------\\

public class LEDSubsystem extends SubsystemBase {

    //-------- DECLARATIONS --------\\

    private AddressableLED leds;

    //-------- CONSTRUCTOR --------\\

    public LEDSubsystem(int port, int length) {
        leds = new AddressableLED(port); //initialization of the AdressableLED
        leds.setLength(length); //Sets the LED Strip length once
    }

    //-------- METHODS --------\\
    
    //call the leds to start the subsystem
    public void start(){
        leds.start();
    }

    //sets the buffer(color strips) for the LED Strip
    public void setBuffer(AddressableLEDBuffer buffer){
        leds.setData(buffer);
    }
    
} // end of class LEDSubsystem