/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import frc.robot.Constants;
//import frc.robot.utilities.ShuffleboardUtility;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.RobotBase;

//import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utilities.SparkMaxWrapper;

//-------- SUBSYSTEM CLASS --------\\

public class FlywheelSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    // -------- DECLARATIONS --------\\

    private static final Logger logger = Logger.getLogger(FlywheelSubsystem.class.getName());
    //private ShuffleboardUtility shuffleboardUtility;

    // motor controllers for the NEO motors on the shooter
    private final CANSparkMax motorLead;
    private final CANSparkMax motor2;

    //private CANPIDController pidcontroller;

    // -------- CONSTRUCTOR --------\\

    public FlywheelSubsystem() {

        // Motor declaration
        // Determines if dealing with a real robot. 
        if(RobotBase.isReal()){ 
            // If true, use hardware
            this.motorLead = new CANSparkMax(Constants.SHOOTER_LEAD_ID, MotorType.kBrushless);
            this.motor2 = new CANSparkMax(Constants.SHOOTER_SLAVE_ID, MotorType.kBrushless);
        } else { 
            // If false, use simulator
            this.motorLead = new SparkMaxWrapper(Constants.SHOOTER_LEAD_ID, MotorType.kBrushless);
            this.motor2 = new SparkMaxWrapper(Constants.SHOOTER_SLAVE_ID, MotorType.kBrushless);
        }

        // Follow lead reverse speed
        if(RobotBase.isReal()){
            motor2.follow(motorLead, true);
        }

        //shuffleboardUtility = ShuffleboardUtility.getInstance();
    }

    // -------- METHODS --------\\

    public double getVoltage() {
        return motorLead.getBusVoltage();
    }

    public void setVoltage(double outputVolts) {
        motorLead.setVoltage(outputVolts);
    }

    public double getSpeed() {
        return motorLead.getEncoder().getVelocity();
        
    }

}
// end of class ShooterSubsystem