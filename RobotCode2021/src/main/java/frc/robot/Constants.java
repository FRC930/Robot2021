/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

import java.util.logging.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * 
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // [-------------------MOTOR IDS--------------------------]

    public static final int INTAKE_ID = 17;
    public static final int COLOR_WHEEL_ID = 333;


    // [-------------------MOTOR IDS--------------------------]

    // [-------------------PISTON IDS-------------------------]

    public static final int SHIFTER_SOLENOID_ID = 2;

    public static final int COMPRESSOR_PORT = 0;

    // [-------------------PISTON IDS-------------------------]

    // [-------------------ROBORIO PORT IDS---------------------------]

	public static final int HOPPER_ENCODER_PORT_ID = 1; 
    public static final int CLIMBER_ENCODER_PORT_ID = 2;

    // [-------------------ROBORIO PORT IDS---------------------------]

    // [-------------------DRIVE------------------------]

    public static final double MOTOR_RAMP_RATE = 0.75;//0.5;

    // [-------------------DRIVE------------------------]

    // [-------------------SPEEDS--------------------------]

    public static final double HOPPER_DEFAULT_SPEED = 0.3;
    public static final double HOPPER_REVERSE_SPEED = 0.35;

    public static final double HOPPER_SHOOTING_SPEED = 0.7;
    public static final double TOWER_SPEED = 1.0;

    public static final double TOWER_REVERSE_SPEED = -0.5;
    public static final double KICKER_SPEED = 1.0;
    public static final double INTAKE_SPEED = 0.6;
    public static final double FLYWHEEL_TELEOP_SPEED = 0.5;

    public static final double FLYWHEEL_AUTON_SPEED = 0.8;
    public static final double CLIMBER_EXTEND_SPEED = 0.1;
    public static final double CLIMBER_RETRACT_SPEED = -0.1;

    // [-------------------SPEEDS--------------------------]

    // [-------------------LIMITS--------------------------]

    public static final double ENCODER_ROTATION_LIMIT = 190;
    public static final double CLIMBER_LIMIT = -2.4;

    // [-------------------LIMITS--------------------------]

    // [-------------------TURRET--------------------------]

    // When the turret encoder is reset, the turret faces forward and the encoder is
    // reset to 180 degrees. These units are in raw values.
    public static final double UPPER_LIMIT = 0.697;
    public static final double LOWER_LIMIT = 0.335;

    public static final double TURRET_MAX_SPEED = 0.6;
    public static final double TURRET_MAX_SET_POSITION_SPEED = 0.4;

    public static final double TURRET_P = 0.025;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0008;

    public static final double TURRET_SET_POSITION_P = 2.5;
    public static final double TURRET_SET_POSITION_I = 0.0;
    public static final double TURRET_SET_POSITION_D = 0.0;

    // encoder positions for setting turret to one of four directions
    public static final double TURRET_BACK_POSITION = 0.635;  // Toward intake
    public static final double TURRET_FRONT_POSITION = 0.383;  // Toward Back of robot
    public static final double TURRET_RIGHT_POSITION = 0.51;
    public static final double TURRET_LEFT_POSITION = 0.256;

    public static final double FRONT_LEFT_POSITION = 0.3195;
    public static final double FRONT_RIGHT_POSITION = 0.4465;
    public static final double BACK_RIGHT_POSITION = 0.5725;

    // speed used for turning the turret
    public static final double TURRET_TURNING_SPEED = 0.4;

    // deadband for the turret joystick
    public static final double JOYSTICK_TURRET_DEADBAND = 0.1;

    // deadband for the turret set position commands
    public static final double TURRET_DEADBAND = 0.01;

    // [-------------------TURRET--------------------------]

    // [-------------------ANGLES--------------------------]

    public static final double FLYWHEEL_LOWER_ANGLE = 31.4;
    public static final double FLYWHEEL_UPPER_ANGLE = 39.0;

    // [-------------------ANGLES--------------------------]

    // [-------------------STATES--------------------------]

    public static final boolean INTAKE_PISTONS_UP = false;
    public static final boolean INTAKE_PISTONS_DOWN = !INTAKE_PISTONS_UP;
    public static final int LIMELIGHT_LEDS_ON = 3;
    public static final int LIMELIGHT_LEDS_OFF = 1;

    // [--------------------AUTO--------------------------]

    public static final double KSVOLTS = 0.67;
    public static final double KVVOLT = 2.31;
    public static final double KAVOLT = 0.0844; // this is in seconds squared per meter
    public static final double KMAXSPEED = Units.feetToMeters(16.2); //in meters per second
    public static final double KMAXACCELERATION = 2; //in meters per seconds squared 
    public static final double KMAXANGULARSPEED = Math.PI;
    //gyro values
    public static final double KRAMSETEB = 2;
    public static final double KRAMSETEZETA = 0.7;

    // Track width of our robot
    public static final double KTRACKWIDTH = 0.5715; // in meters
    public static final double KPDRIVEVEL = 1.49;
    // AUTO code values---------------------------------------]

    // DRIVE Constants --------------------------------]

    //Drivetrain
    public static final double DRIVE_DEADBAND_JOYSTICK = 0.000125;
    public static final double DRIVE_TURNING_MULTIPLIER = 0.5;

    //Swervedrive
    //TODO: Add constants for deadbands.

    // DRIVE Constants --------------------------------]

    //[-------------------LOGGER--------------------------]

    public static final Level LOG_LEVEL_FINE = Level.FINE;
    public static final Level LOG_LEVEL_FINER = Level.FINER;
    public static final Level LOG_LEVEL_FINEST = Level.FINEST;
    public static final Level LOG_LEVEL_INFO = Level.INFO;
    public static final Level LOG_LEVEL_WARNING = Level.WARNING; 

    //[-------------------LOGGER--------------------------]

    // OFFSET
    
	public static final double KICKER_ENCODER_OFFSET = 0;
    
} // end of constants