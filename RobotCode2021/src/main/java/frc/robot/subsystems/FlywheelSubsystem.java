/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Units;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utilities.SparkMaxWrapper;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h4>FlywheelSubsystem</h4>
 * Subsystem class to manage the flywheel and all related hardware.
 */
public class FlywheelSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    // -------- DECLARATIONS --------\\

    private static final Logger logger = Logger.getLogger(FlywheelSubsystem.class.getName());
    // motor controllers for the NEO motors on the shooter
    private final CANSparkMax motorLead;
    private final CANSparkMax motor2;

    private final CANEncoder encoder;

    /** Flywheel motor speed */
    private double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2000.0);  

    // -------- CONSTRUCTOR --------\\

    /**
     * <h4>FlywheelSubsystem</h4>
     * Creates a subsystem class to manage the flywheel and all related hardware.
     * 
     * @param shooterLeadID ID of the shooter lead motor
     * @param shooterSlaveID ID of the shooter "slave" motor
     */
    public FlywheelSubsystem(int shooterLeadID, int shooterSlaveID) {

        // Motor declaration
        // Determines if dealing with a real robot. 
        if(RobotBase.isReal()){ 
            // If true, use hardware
            this.motorLead = new CANSparkMax(shooterLeadID, MotorType.kBrushless);
            this.motor2 = new CANSparkMax(shooterSlaveID, MotorType.kBrushless);
        } else { 
            // If false, use simulator
            this.motorLead = new SparkMaxWrapper(shooterLeadID, MotorType.kBrushless);
            this.motor2 = new SparkMaxWrapper(shooterSlaveID, MotorType.kBrushless);
        }

        // Follow lead reverse speed
        if(RobotBase.isReal()){
            motor2.follow(motorLead, true);
        }
        encoder = motorLead.getEncoder();
    }

    // -------- METHODS --------\\

    /**
     * <h4>getVoltage</h4>
     * telling us the voltage of the motorLead
     * 
     * @return motorLead.getBusVoltage() Gets the voltage of the lead motor
     */
    public double getVoltage() {
        return motorLead.getBusVoltage();
    }

    /**
     * <h4>setVoltage</h4>
     * sets the the motorLead to the outputVolts
     * 
     * @param outputVolts Sets the voltage of the lead motor
     */
    public void setVoltage(double outputVolts) {
        motorLead.setVoltage(outputVolts);
    }

    /**
     * <h4>getSpeed</h4>
     * tells us the velocity of the encoder
     * 
     * @return encoder.getVelocity() Velocity of the encoder
     */
    public double getSpeed() {
        return encoder.getVelocity();
    }

    /**
     * <h4>getRadiansPerSecond</h4>
     * tells us the radians per second of the flywheel
     * 
     * @return kSpinupRadPerSec Flywheel motor speed
     */
    public double getRadiansPerSecond() {
        return this.kSpinupRadPerSec;
    }

    /**
     * <h4>setSpeedRPMs</h4>
     * sets flywheel motor speed (in rpms)
     *
     * @param RPMS
     */
    public void setSpeedRPMs(double RPMS) {
        this.kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(RPMS);
    } 
}
// end of class ShooterSubsystem