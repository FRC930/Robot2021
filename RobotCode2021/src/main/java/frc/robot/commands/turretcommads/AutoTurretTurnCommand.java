/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.turretcommads;

//import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

//-------- COMMAND CLASS --------\\

public class AutoTurretTurnCommand extends CommandBase {
    //You must include logger as a constant variable, and you must have logging in your files
    //private static final Logger logger = Logger.getLogger(AutoTurretTurnCommand.class.getName());
    private double turretPosition;
    private double targetPosition = 0.3565;
    private double speed;
    private TurretSubsystem turretSubsystem;    
    
    public AutoTurretTurnCommand(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
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
        if(Math.abs(turretPosition - targetPosition) > Constants.TURRET_DEADBAND){
            if(turretPosition < targetPosition) {
                speed = -Constants.TURRET_TURNING_SPEED;
            } else if(turretPosition > targetPosition) {
                speed = Constants.TURRET_TURNING_SPEED;
            }    
        } else {
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
