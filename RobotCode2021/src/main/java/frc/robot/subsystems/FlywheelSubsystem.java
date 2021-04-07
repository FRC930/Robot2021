/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Units;

import com.revrobotics.CANEncoder;
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

    private final CANEncoder encoder;

    
    private double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2000.0);  

    // -------- CONSTRUCTOR --------\\

    public FlywheelSubsystem(int SHOOTER_LEAD_ID, int SHOOTER_SLAVE_ID) {

        // Motor declaration
        // Determines if dealing with a real robot. 
        if(RobotBase.isReal()){ 
            // If true, use hardware
            this.motorLead = new CANSparkMax(SHOOTER_LEAD_ID, MotorType.kBrushless);
            this.motor2 = new CANSparkMax(SHOOTER_SLAVE_ID, MotorType.kBrushless);
        } else { 
            // If false, use simulator
            this.motorLead = new SparkMaxWrapper(SHOOTER_LEAD_ID, MotorType.kBrushless);
            this.motor2 = new SparkMaxWrapper(SHOOTER_SLAVE_ID, MotorType.kBrushless);
        }

        // Follow lead reverse speed
        if(RobotBase.isReal()){
            motor2.follow(motorLead, true);
        }
        encoder = motorLead.getEncoder();
        //shuffleboardUtility = ShuffleboardUtility.getInstance();
    }

    // -------- METHODS --------\\
    //  telling us the voltage of the motorLead
    public double getVoltage() {
        return motorLead.getBusVoltage();
    }
    //sets the the motorLead to the outputVolts
    public void setVoltage(double outputVolts) {
        motorLead.setVoltage(outputVolts);
    }
    // tells us the velocity of the encoder
    public double getSpeed() {
        return encoder.getVelocity();
        
    }
    //  tells us the radians per second of the flywheel
    public double getRadiansPerSecond() {
        return this.kSpinupRadPerSec;
    }

     //sets flywheel motor speed (in rpms)
    public void setSpeedRPMs(double RPMS) {
        this.kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(RPMS);
    } 
}
// end of class ShooterSubsystem