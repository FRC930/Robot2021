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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h4>TurretSubsystem
 * Subsystem class that manages the motors that turn the shooter.
 */
public class TurretSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    private static final Logger logger = Logger.getLogger(TurretSubsystem.class.getName());
    // constant used in the conversion from encoder units to degrees
    private final double DEGREE_CONVERSION_NUMBER = .0013889;

    // When the turret encoder is reset, the turret faces forward and the encoder is
    // reset to 180 degrees. These units are in raw values.
    private final double UPPER_LIMIT = 0.697;
    private final double LOWER_LIMIT = 0.335;
    private final double TURRET_FRONT_POSITION = 0.383; // back of robot
    private final double TURRET_BACK_POSITION = 0.635;  // front robot intake
    private final double TURRET_RIGHT_POSITION = 0.51;
    private final double TURRET_LEFT_POSITION = 0.256;
      // encoder positions for setting turret to one of four directions
    private final double FRONT_LEFT_POSITION = 0.3195;
    private final double FRONT_RIGHT_POSITION = 0.4465;
    private final double BACK_RIGHT_POSITION = 0.5725;

    private final double TURRET_MAX_SPEED = 0.6;
    private final double TURRET_MAX_SET_POSITION_SPEED = Constants.TURRET_MAX_SET_POSITION_SPEED;

    // -------- DECLARATIONS --------\\

    // The motor controller that will control the turret
    private WPI_TalonSRX turretMotor;
    private DutyCycleEncoder encoder;
    private double encoderPosition;

    // -------- CONSTRUCTOR --------\\

    /**
     * <h4>TurretSubsystem
     * Creates a subsystem class that manages the motors that turn the shooter.
     * 
     * @param turretID ID of the turret motor
     * @param encoderPortID ID of the encoder
     */
    public TurretSubsystem(int turretID, int encoderPortID) {
        this.turretMotor = new WPI_TalonSRX(turretID);

        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            this.encoder = new DutyCycleEncoder(encoderPortID);
        }
        logger.log(Constants.LOG_LEVEL_INFO, "Starting TurretSubsystem");
    }

    // -------- METHODS --------\\

    /**
     * <h4>setSpeed</h4>
     * Sets the speed of the turret motor.
     * 
     * @param speed
     */
    public void setSpeed(double speed) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            encoderPosition = encoder.get();
        }

        SmartDashboard.putNumber("Turret speed unclamped", speed);
        SmartDashboard.putNumber("Turret pos", encoderPosition);

        speed = clamp(speed); // Clamps the speed down to max if it is above

        // If the encoderPosition is beyond either limit, the speed of the motor will be set to 0 to prevent it from going past.
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

    /**
     * <h4>getSpeed</h4>
     * Gets the speed of the turret motor.
     * 
     * @return turretMotor.getMotorOutputPercent()
     */
    public double getSpeed() {
        return turretMotor.getMotorOutputPercent();
    }

    /**
     * <h4>unitsToDegrees</h4>
     * Converts encoder units to degrees
     * 
     * @param units Encoder units
     */
    public double unitsToDegrees(double units) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            return this.encoder.get() / DEGREE_CONVERSION_NUMBER;
        } else {
            return 0.0;
        }
    }

    /**
     * <h4>getEncoderPosition</h4>
     * Returns the current encoder position in degrees
     * 
     * @return untisToDegrees(this.encoder.get())
     */
    public double getEncoderPosition() {
        return unitsToDegrees(this.encoder.get());
    }

    /**
     * <h4>getRawEncoderPosition</h4>
     * Returns the current encoder position in encoder units
     * 
     * @return this.encoder.get() or 0.0
     */
    public double getRawEncoderPosition() {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            return this.encoder.get();
        } else {
            return 0.0;
        }
    }

    /**
     * <h4>clamp</h4>
     * Helper method to "clamp" down the speed to a positive and negative max speed.
     * Basically ensures that a speed doesn't exceed the maxes.
     * 
     * @return speed
     */
    private double clamp(double speed) {
        if (speed > TURRET_MAX_SPEED) {
            speed = TURRET_MAX_SPEED;
        } else if (speed < -TURRET_MAX_SPEED) {
            speed = -TURRET_MAX_SPEED;
        }
        return speed;
    }

    public double getFrontPosition() {
        return TURRET_FRONT_POSITION;
    }

    public double getRightPosition() {
        return TURRET_RIGHT_POSITION;
    }

    public double getBackPosition() {
        return TURRET_BACK_POSITION;
    }

    public double getLeftPosition() {
        return TURRET_LEFT_POSITION;
    }

    public double getFrontLeftPosition() {
        return FRONT_LEFT_POSITION;
    }

    public double getFrontRightPosition() {
        return FRONT_RIGHT_POSITION;
    }

    public double getBackRightPosition() {
        return BACK_RIGHT_POSITION;
    }

} // end of class TurretSubsystem
