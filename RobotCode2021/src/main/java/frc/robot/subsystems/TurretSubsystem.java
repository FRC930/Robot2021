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

    // When the turret encoder is reset, the turret faces forward and the encoder is
    // reset to 180 degrees. These units are in raw values.
    private final double UPPER_LIMIT = 0.697;
    private final double LOWER_LIMIT = 0.335;
    private final double TURRET_FRONT_POSITION = 0.383; // back of r0bot
    private final double TURRET_AUTO_TURN_POSITION = 0.3565;
    private final double TURRET_BACK_POSITION = 0.635;  // front robot intake
    private final double TURRET_RIGHT_POSITION = 0.51;
    private final double TURRET_LEFT_POSITION = 0.256;
    // encoder positions for setting turret to one of four directions
    private final double FRONT_LEFT_POSITION = 0.3195;
    private final double FRONT_RIGHT_POSITION = 0.4465;
    private final double BACK_RIGHT_POSITION = 0.5725;

    // speed used for turning the turret
    private final double TURRET_TURNING_SPEED = 0.4;

    // deadband for the turret joystick
    private final double JOYSTICK_TURRET_DEADBAND = 0.1;

    // deadband for the turret set position commands
    private final double TURRET_DEADBAND = 0.01;

    private final double TURRET_MAX_SPEED = 0.6;
    private final double TURRET_MAX_SET_POSITION_SPEED = 0.4;

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
        SmartDashboard.putNumber("Turret pos", encoderPosition);
        speed = clamp(speed);
        if (encoderPosition > UPPER_LIMIT) {
            if (speed < 0) {
                speed = 0;
            }
        } else if (encoderPosition < LOWER_LIMIT) {
            if (speed > 0) {
                speed = 0;
            }
        } else if (encoderPosition == 0) {
            // to resolve intermident 0 encoder readings
            // TODO: look into the encoder hardware for problems

            speed = 0;
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
        if (speed > TURRET_MAX_SPEED) {
            speed = TURRET_MAX_SPEED;
        } else if (speed < -TURRET_MAX_SPEED) {
            speed = -TURRET_MAX_SPEED;
        }
        return speed;
    }

    public double getTurretTurningSpeed(){
        return TURRET_TURNING_SPEED;
    }

    public double getJoystickTurretDeadband(){
        return JOYSTICK_TURRET_DEADBAND;
    }

    public double getTurretDeadband(){
        return TURRET_DEADBAND;
    }

    public double getFrontPosition(){
        return TURRET_FRONT_POSITION;
    }

    public double getRightPosition(){
        return TURRET_RIGHT_POSITION;
    }

    public double getBackPosition(){
        return TURRET_BACK_POSITION;
    }

    public double getLeftPosition(){
        return TURRET_LEFT_POSITION;
    }

    public double getFrontLeftPosition(){
        return FRONT_LEFT_POSITION;
    }

    public double getFrontRightPosition(){
        return FRONT_RIGHT_POSITION;
    }

    public double getBackRightPosition(){
        return BACK_RIGHT_POSITION;
    }

    public double getTurretAutoTurnPosition() {
        return TURRET_AUTO_TURN_POSITION;
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
