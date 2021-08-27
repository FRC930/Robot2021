/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.turretcommads;

import java.util.logging.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h4>JoystickTurretCommand</h4> Moves the turret using the coDriver joystick
 */
public class JoystickTurretCommand extends CommandBase {

    // -------- DECLARATIONS --------\\

    private static final Logger logger = Logger.getLogger(JoystickTurretCommand.class.getName());

    private TurretSubsystem turretSubsystem;
    private Joystick coDriver;
    private int coDriverAxis;
    private double stickX;

    // -------- CONSTRUCTOR --------\\
    /**
     * Creates an instance of the command.
     * 
     * @param turretSubsystem valid instance of the TurretSubsystem
     * @param coDriver        valid instance of the coDriver joystick
     * @param coDriverAxis    index for the joystick axis
     */
    public JoystickTurretCommand(TurretSubsystem turretSubsystem, Joystick coDriver, int coDriverAxis) {
        this.turretSubsystem = turretSubsystem;
        this.coDriver = coDriver;
        this.coDriverAxis = coDriverAxis;
        this.stickX = 0;

        addRequirements(turretSubsystem);
    }

    // -------- METHODS --------\\

    @Override
    public void execute() {

        double speed;

        stickX = coDriver.getRawAxis(coDriverAxis);

        logger.log(Level.INFO, "stickX = " + stickX);
        // Calculates if the joystick is past it's deadband
        if (Math.abs(stickX) > turretSubsystem.getJoystickTurretDeadband()) {
            logger.log(Level.INFO,
                    "stickX >  Constants.JOYSTICK_TURRET_DEADBAND(" + turretSubsystem.getJoystickTurretDeadband() + ")");
            // Raises the joystick value to the power of 3
            speed = Math.pow(stickX, 3) * 0.5;
        } else {
            logger.log(Level.INFO,
                    "stickX <  Constants.JOYSTICK_TURRET_DEADBAND(" + turretSubsystem.getJoystickTurretDeadband() + ")");
            speed = 0;
        }

        SmartDashboard.putNumber("co driver axis", stickX);
        SmartDashboard.putNumber("joystick turret speed", speed);

        // we pass in a negative speed to match Kyle's joystick-+
        turretSubsystem.setSpeed(-speed);

    } // end of class execute()

    @Override
    public boolean isFinished() {
        return false;
    }

} // end of class JoystickTurretCommand