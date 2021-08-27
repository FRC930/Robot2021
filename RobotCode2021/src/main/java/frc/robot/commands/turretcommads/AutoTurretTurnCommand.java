/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.turretcommads;

import java.util.logging.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h4>AutoTurretTurnCommand</h4> Moves the turret pointing away from the intake.
 */
public class AutoTurretTurnCommand extends CommandBase {
    // You must include logger as a constant variable, and you must have logging in
    // your files
    private static final Logger logger = Logger.getLogger(AutoTurretTurnCommand.class.getName());
    private double turretPosition;
    private double speed;
    private TurretSubsystem turretSubsystem;

    /**
     * Creates a instance of the Command.
     * 
     * @param turretSubsystem valid instance of the TurretSubsystem
     */
    public AutoTurretTurnCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        turretPosition = 0;
        speed = 0;
        addRequirements(turretSubsystem);
    }

    // -------- METHODS --------\\

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        turretPosition = turretSubsystem.getRawEncoderPosition();
        // Calculates the direction the turret needs to move and turns it at a constant
        // speed.
        if (Math.abs(turretPosition - turretSubsystem.getTurretAutoTurnPosition()) > turretSubsystem.getTurretDeadband()) {
            if (turretPosition < turretSubsystem.getTurretAutoTurnPosition()) {
                speed = -turretSubsystem.getTurretTurningSpeed();
            } else if (turretPosition > turretSubsystem.getTurretAutoTurnPosition()) {
                speed = turretSubsystem.getTurretTurningSpeed();
            }
        } else {
            speed = 0;
        }
        logger.log(Level.INFO, "Turret speed = " + speed);
        turretSubsystem.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Calls to end the command once the turret stops moving
        return speed == 0;
    }
} // end of class TurretBackCommand
