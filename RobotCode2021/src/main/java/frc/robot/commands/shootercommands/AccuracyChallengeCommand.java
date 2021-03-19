/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.utilities.DistanceMath;

import frc.robot.commands.shootercommands.pistoncommands.*;

import frc.robot.Constants;

import frc.robot.subsystems.LimelightSubsystem;                                                                                                                                                                     

//-------- COMMAND CLASS --------\\

public class AccuracyChallengeCommand extends CommandBase {

    //-------- CONSTANTS --------\\

    //You must include logger as a constant variable, and you must have logging in your files
    /*private final Logger logger = Logger.getLogger(this.getClass().getName());*/

    /*private final Enum Zone {

    }*/

    //-------- DECLARATIONS --------\\

    //NOTE: You variables must be private AND must be named in a camel case format!
    //Here is an example variable
    //One of your variables MUST be a subsystem
    private FlywheelSubsystem _myFlywheelSubsystem;    //NOTE: Do not include the m_
    private FlywheelPistonSubsystem _myFlywheelPistonSubsystem;
    private LimelightSubsystem _myLimelightSubsystem;
    
    //-------- CONSTRUCTOR --------\\

    public AccuracyChallengeCommand(FlywheelSubsystem _myFlywheelSubsystem,
                                    FlywheelPistonSubsystem _myFlywheelPistonSubsystem,
                                    LimelightSubsystem _myLimelightSubsystem){
        this._myLimelightSubsystem = _myLimelightSubsystem;
        this._myFlywheelSubsystem = _myFlywheelSubsystem;
        this._myFlywheelPistonSubsystem = _myFlywheelPistonSubsystem;
        addRequirements(_myFlywheelSubsystem, _myFlywheelPistonSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   

    }

    // Called every time the scheduler runs while the command is scheduled.
    //NOTE: DO NOT INCLUCE EXECUTE INSIDE SEQUENTIAL AND PARALLEL COMMANDS!!!!
    @Override
    public void execute() {
        double distance = DistanceMath.getDistY(_myLimelightSubsystem.getVerticleOffset());
        if ( 83 < distance && distance < 85 ) {   // Green Zone (Back)
            _myFlywheelSubsystem.setSpeedRPMs(2000.0); //needs to be changed!
        } else if ( 143 < distance && distance < 145 ) {   // Yellow Zone (Back)
            _myFlywheelSubsystem.setSpeedRPMs(2170.0);
        } else if ( 203 < distance && distance < 205 ) {   // Blue Zone (Back)
            _myFlywheelSubsystem.setSpeedRPMs(3145.0);
        } else if ( 215 < distance && distance < 218 ) {   // Red Zone (Front)  :)
            _myFlywheelSubsystem.setSpeedRPMs(3155.0);
        }
    }

/*
    public void execute() {
        double distance = DistanceMath.getDistY(_myLimelightSubsystem.getVerticleOffset());
        switch(distance) {
            case 0:

            break;
        }
    }
*/

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
} // end of class AccuracyChallengeCommand