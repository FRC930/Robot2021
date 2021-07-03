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

import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

//-------- COMMAND CLASS --------\\

public class SetTurretPositionCommand extends CommandBase {

    //You must include logger as a constant variable, and you must have logging in your files
    private static final Logger logger = Logger.getLogger(SetTurretPositionCommand.class.getName());

    private double turretPosition;
    private double speed;
    private double turretTarget;

    private TurretSubsystem turretSubsystem;    
    
    public SetTurretPositionCommand(TurretSubsystem turretSubsystem, String TurretPosition){
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
        if(TurretPosition.toLowerCase().equals("front")){
        turretTarget=turretSubsystem.getFrontPosition();
        }
        else if(TurretPosition.toLowerCase().equals("right")){
        turretTarget=turretSubsystem.getRightPosition();
        }
        else if(TurretPosition.toLowerCase().equals("back")){
            turretTarget=turretSubsystem.getBackPosition();
        }
        else if(TurretPosition.toLowerCase().equals("left")){
            turretTarget=turretSubsystem.getLeftPosition();
        }
        else if(TurretPosition.toLowerCase().equals("frontLeft")){
        turretTarget=turretSubsystem.getFrontLeftPosition();
        }
        else if(TurretPosition.toLowerCase().equals("frontRight")){
            turretTarget=turretSubsystem.getFrontRightPosition();
        }
        else if(TurretPosition.toLowerCase().equals("backRight")){
            turretTarget=turretSubsystem.getBackRightPosition();
        }
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        turretPosition = turretSubsystem.getRawEncoderPosition();

        logger.log(Level.INFO, "targetPosition = " + turretTarget);
        logger.log(Level.INFO, "turretPosition = " + turretPosition);

        if(Math.abs(turretPosition - turretTarget) > Constants.TURRET_DEADBAND){
            logger.log(Level.INFO, "| turretPosition - targetPosition | >  Constants.TURRET_DEADBAND("+Constants.TURRET_DEADBAND+")");
            if(turretPosition < turretTarget) {
                logger.log(Level.INFO, "seting 'speed' to -Constants.TURRET_TURNING_SPEED("+-Constants.TURRET_TURNING_SPEED+")");
                speed = -Constants.TURRET_TURNING_SPEED;
            } else if(turretPosition > turretTarget) {
                logger.log(Level.INFO, "seting 'speed' to Constants.TURRET_TURNING_SPEED("+Constants.TURRET_TURNING_SPEED+")");
                speed = Constants.TURRET_TURNING_SPEED;
            }  
        } else {
            logger.log(Level.INFO, "| turretPosition - targetPosition | <  Constants.TURRET_DEADBAND("+Constants.TURRET_DEADBAND+")");
            speed = 0;
        }
        
        turretSubsystem.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return speed == 0;
    }
} // end of class TurretBackCommand 

