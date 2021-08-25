/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>TowerSubsystem</h3>
 * 
 * TowerSubsystem controls the tower on the robot. This class uses a Victor to
 * control the rotation of the tower. Speed can be set by calling the
 * {@link #setSpeed(double) setSpeed} method.
 */
public class TowerSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    private static final Logger logger = Logger.getLogger(TowerSubsystem.class.getName());
    private final double TOWER_SPEED = 1.0;
    private final double TOWER_REVERSE_SPEED = -0.5;
    
    //-------- DECLARATIONS --------\\

    // VictorSPX is a motor controller that makes the conveor belt Take's the power
    // cell up to the shooter
    private WPI_VictorSPX towerMotor;

    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>TowerSubsystem</h3>
     * 
     * This constructor takes an integer as a parameter and creates a new
     * {@link com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX Victor} for the
     * control of the tower
     * 
     * @param towerID the CAN ID of the tower motor
     */
    public TowerSubsystem(int towerID) {
        towerMotor = new WPI_VictorSPX(towerID);
    }

    // -------- METHODS --------\\

    /**
     * <h3>setSpeed</h3>
     * 
     * This method sets the speed of the motor
     * 
     * @param speed the speed at which the motor should run
     */
    public void setSpeed(double speed) {
        logger.entering(TowerSubsystem.class.getName(), "setSpeed");
        logger.log(Constants.LOG_LEVEL_FINE, "motorSpeed: " + -speed);

        towerMotor.set(ControlMode.PercentOutput, -speed);

        logger.exiting(TowerSubsystem.class.getName(), "setSpeed");
    }

    /**
     * <h3>getSpeed</h3>
     * 
     * This method returns the speed that the motor should be running at
     * 
     * @return the current speed of the motor
     */
    public double getSpeed() {
        logger.entering(TowerSubsystem.class.getName(), "getSpeed");
        logger.log(Constants.LOG_LEVEL_FINER, "motorSpeed: " + towerMotor.getMotorOutputPercent());
        logger.exiting(TowerSubsystem.class.getName(), "getSpeed");

        return towerMotor.getMotorOutputPercent();
    }

    /**
     * <h3>stopMotor</h3>
     * 
     * This method sets the speed of the motor to zero, effectively stopping the
     * motor.
     */
    public void stopMotor() {
        towerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /** @return {@link #TOWER_SPEED} constant */
    public double getTowerSpeed(){
        return TOWER_SPEED;
    }

    /** @return {@link #TOWER_REVERSE_SPEED} constant */
    public double getTowerReverseSpeed(){
        return TOWER_REVERSE_SPEED;
    }
    
} //end of class TowerSubsystem
