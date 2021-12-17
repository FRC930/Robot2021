/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
  
    // [-------------------LIMITS--------------------------]

    //public static final double ENCODER_ROTATION_LIMIT = 190;

    // [-------------------TURRET--------------------------]

    public static final double TURRET_P = 0.025;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0008;

    //[-------------------LOGGER--------------------------]

    public static final Level LOG_LEVEL_FINE = Level.FINE;
    public static final Level LOG_LEVEL_FINER = Level.FINER;
    public static final Level LOG_LEVEL_FINEST = Level.FINEST;
    public static final Level LOG_LEVEL_INFO = Level.INFO;
    public static final Level LOG_LEVEL_WARNING = Level.WARNING; 
    
} // end of constants