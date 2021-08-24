/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

import java.util.logging.Logger;

import frc.robot.Constants;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h4>IntakePistonSubsystem</h4>
 * This subsystem controlls the solenoids for the intake pistons.
 */
public class IntakePistonSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    private static final Logger logger = Logger.getLogger(IntakePistonSubsystem.class.getName());
    // Intake piston state constants
    private final boolean INTAKE_PISTONS_UP = false;
    private final boolean INTAKE_PISTONS_DOWN = !INTAKE_PISTONS_UP;

    // -------- DECLARATIONS --------\\

    private Solenoid intakePistonController;

    // -------- CONSTRUCTOR --------\\
    /**
     * Instantiates solenoid controller {@link #intakePistonController}.
     * @param INTAKE_SOLENOID_ID solenoid ID
     */
    public IntakePistonSubsystem(int INTAKE_SOLENOID_ID) {
        intakePistonController = new Solenoid(INTAKE_SOLENOID_ID);
    }

    // -------- METHODS --------\\
    /**
     * Sets the solenoids for the pistons.
     * @param state piston/solenoid state.
     */
    public void setIntakePistonState(boolean state) {
        intakePistonController.set(state);

        logger.log(Constants.LOG_LEVEL_INFO, "setIntakePistonState: " + state);
    }

    /**
     * Get the piston state.
     * @return {@link #intakePistonController}
     */
    public boolean getIntakePistonState() {
        logger.log(Constants.LOG_LEVEL_INFO, "getIntakePistonState: " + intakePistonController.get());

        return intakePistonController.get();
    }

    /**
     * 
     * @return {@link #INTAKE_PISTONS_UP} state
     */
    public boolean getIntakePistonUpState() {
        return INTAKE_PISTONS_UP;
    }

    /**
     * 
     * @return {@link #INTAKE_PISTONS_DOWN} state
     */
    public boolean getIntakePistonDownState() {
        return INTAKE_PISTONS_DOWN;
    }

} // end of class IntakePistonSubsystem
