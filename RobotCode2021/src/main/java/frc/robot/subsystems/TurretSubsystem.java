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

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\

public class TurretSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    private static final Logger logger = Logger.getLogger(TurretSubsystem.class.getName());
    // constant used in the conversion from encoder units to degrees
    private final double DEGREE_CONVERSION_NUMBER = .0013889;

    // -------- DECLARATIONS --------\\

    // The motor controller that will control the turret
    private WPI_TalonSRX turretMotor;
    private DutyCycleEncoder encoder;
    //private ShuffleboardUtility shuffleboardUtility;
    private double encoderPosition;

    // -------- CONSTRUCTOR --------\\

    public TurretSubsystem(int TURRET_ID, int ENCODER_PORT_ID) {
        this.turretMotor = new WPI_TalonSRX(TURRET_ID);

        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            this.encoder = new DutyCycleEncoder(ENCODER_PORT_ID);
        }
        //shuffleboardUtility = ShuffleboardUtility.getInstance();
        logger.log(Constants.LOG_LEVEL_INFO, "Starting TurretSubsystem");
    }

    // -------- METHODS --------\\

    public void setSpeed(double speed) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            encoderPosition = encoder.get();
        }

        //encoderPosition = 0.0;


        SmartDashboard.putNumber("Turret speed unclamped", speed);
        speed = clamp(speed);
        if (encoderPosition > Constants.UPPER_LIMIT) {
            if (speed < 0) {
                speed = 0;
            }
        } else if (encoderPosition < Constants.LOWER_LIMIT) {
            if (speed > 0) {
                speed = 0;
            }
        }

        this.turretMotor.set(ControlMode.PercentOutput, speed);
        
        logger.log(Constants.LOG_LEVEL_FINER, "Set speed to " + getSpeed());
    }

    public double getSpeed() {
        return turretMotor.getMotorOutputPercent();
    }

    // converts encoder units to degrees
    public double unitsToDegrees(double units) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            return this.encoder.get() / DEGREE_CONVERSION_NUMBER;
        } else {
            return 0.0;
        }
    }

    // returns the current encoder position in degrees
    public double getEncoderPosition() {
        // return this.turretMotor.getSelectedSensorPosition();
        return unitsToDegrees(this.encoder.get());
    }

    public double getRawEncoderPosition() {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            return this.encoder.get();
        } else {
            return 0.0;
        }
    }

    private double clamp(double speed) {
        if (speed > Constants.TURRET_MAX_SPEED) {
            speed = Constants.TURRET_MAX_SPEED;
        } else if (speed < -Constants.TURRET_MAX_SPEED) {
            speed = -Constants.TURRET_MAX_SPEED;
        }
        return speed;
    }
    // @Override
    // public void periodic() {
    //     // TODO Auto-generated method stub
    //     System.out.println(getRawEncoderPosition());
    //     super.periodic();
    // }

    // @Override
    // public void periodic() {
    //     // shuffleboardUtility.setTurretSpeed(getSpeed());
    //     // shuffleboardUtility.setTurretEncoderPosition(getEncoderPosition());
    // }
} // end of class TurretSubsystem
