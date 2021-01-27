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

import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\

public class KickerSubsystem extends SubsystemBase {
  
    //-------- CONSTANTS --------\\

    private static final Logger logger = Logger.getLogger(KickerSubsystem.class.getName());

    //-------- DECLARATIONS --------\\

    // VictorSPX is a motor controller that makes the conveor belt  Take's the power cell up to the shooter
    private WPI_VictorSPX kickerMotor;  
    private DutyCycleEncoder encoder;

    //-------- CONSTRUCTOR --------\\

    public KickerSubsystem() {
        kickerMotor = new WPI_VictorSPX(Constants.KICKER_ID);  
        if(RobotBase.isReal()){
            this.encoder = new DutyCycleEncoder(Constants.HOPPER_ENCODER_PORT_ID);
            kickerMotor.setInverted(true);
        }
    }   
    
    //-------- METHODS --------\\

    //sets the speed to the tower motor 
    public void setSpeed(double speed) {
        logger.entering(KickerSubsystem.class.getName(), "setSpeed()");
        logger.log(Constants.LOG_LEVEL_FINER, "motorSpeed: " + speed);

        kickerMotor.set(ControlMode.PercentOutput, speed);

        logger.exiting (KickerSubsystem.class.getName(), "setSpeed()");
    }

    // returns the speed from the tower motor
    public double getSpeed() { 
        logger.entering(KickerSubsystem.class.getName(), "getSpeed()");
        logger.log(Constants.LOG_LEVEL_FINER, "motorSpeed: " + kickerMotor.getMotorOutputPercent());
        logger.exiting (KickerSubsystem.class.getName(), "getSpeed()");
        
        return kickerMotor.getMotorOutputPercent();
    }

    public double getEncoder(){
        if(RobotBase.isReal()){
            return encoder.get();
        }
        else {
            return 0.0;
        }
    }

    //stops the motor
    public void stopMotor() {
        kickerMotor.set(ControlMode.PercentOutput, 0.0);
    }

} //end of class KickerSubsystem