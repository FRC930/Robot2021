/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

    // [-------------------SPEEDS--------------------------]

    public static final double HOPPER_DEFAULT_SPEED = 0.3;
    public static final double HOPPER_REVERSE_SPEED = 0.35;

    public static final double HOPPER_SHOOTING_SPEED = 0.7;
    public static final double TOWER_SPEED = 1.0;

    public static final double TOWER_REVERSE_SPEED = -0.5;
    public static final double INTAKE_SPEED = 0.6; 
    public static final double FLYWHEEL_TELEOP_SPEED = 0.5;

    public static final double FLYWHEEL_AUTON_SPEED = 0.8;
    public static final double CLIMBER_EXTEND_SPEED = 0.1;
    public static final double CLIMBER_RETRACT_SPEED = -0.1;
  
    // [-------------------LIMITS--------------------------]

    //public static final double ENCODER_ROTATION_LIMIT = 190;

    // [-------------------TURRET--------------------------]

    public static final double TURRET_MAX_SET_POSITION_SPEED = 0.4;

    public static final double TURRET_P = 0.025;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0008;

    // speed used for turning the turret
    public static final double TURRET_TURNING_SPEED = 0.4;

    // deadband for the turret joystick
    public static final double JOYSTICK_TURRET_DEADBAND = 0.1;

    // deadband for the turret set position commands
    public static final double TURRET_DEADBAND = 0.01;

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

    //Swervedrive
    //TODO: Add constants for deadbands.
        // but not here. instead in driveSubsystem

    //[-------------------LOGGER--------------------------]

    public static final Level LOG_LEVEL_FINE = Level.FINE;
    public static final Level LOG_LEVEL_FINER = Level.FINER;
    public static final Level LOG_LEVEL_FINEST = Level.FINEST;
    public static final Level LOG_LEVEL_INFO = Level.INFO;
    public static final Level LOG_LEVEL_WARNING = Level.WARNING; 
    
} // end of constants