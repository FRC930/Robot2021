
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem.SolenoidValues;
import frc.robot.utilities.DistanceMath;

import frc.robot.Constants;
import frc.robot.commands.shootercommands.flywheelcommands.DefaultFlywheelCommand;
import frc.robot.commands.shootercommands.pistoncommands.FullExtendFlywheelPistonCommand;
import frc.robot.commands.shootercommands.pistoncommands.FullExtendFlywheelPistonCommand;
import frc.robot.commands.shootercommands.pistoncommands.FullRetractFlywheelPistonCommand;
import frc.robot.commands.shootercommands.pistoncommands.FullRetractFlywheelPistonCommand;

//-------- COMMAND CLASS --------\\

public class AccuracyChallengeCommand extends SequentialCommandGroup {

    //-------- CONSTANTS --------\\
    //private final Logger logger = Logger.getLogger(this.getClass().getName());
    private final int GREEN_START_ZONE = 83;
    private final int GREEN_END_ZONE = 85;
    private final double GREEN_SPEED = 500;

    private final int YELLOW_START_ZONE = 143;
    private final int YELLOW_END_ZONE = 145;
    private final double YELLOW_SPEED = 2170.0;

    private final int BLUE_START_ZONE = 203;
    private final int BLUE_END_ZONE = 205;
    private final double BLUE_SPEED = 3150.0;

    private final int RED_START_ZONE = 215;
    private final int RED_END_ZONE = 218;
    private final double RED_SPEED = 3150.0;


    //-------- DECLARATIONS --------\\

    private FlywheelSubsystem mFlywheelSubsystem; 
    private FlywheelPistonSubsystem mFlywheelPistonSubsystem;
    private LimelightSubsystem mLimelightSubsystem;
    private DefaultFlywheelCommand mDefaultFlywheelCommand;

    private FullExtendFlywheelPistonCommand mFullExtendFlywheelPistonCommand;
    private FullRetractFlywheelPistonCommand mFullRetractFlywheelPistonCommand;
    
    //-------- CONSTRUCTOR --------\\

    public AccuracyChallengeCommand(FlywheelSubsystem _mFlywheelSubsystem,
                                    FlywheelPistonSubsystem _mFlywheelPistonSubsystem,
                                    LimelightSubsystem _mLimelightSubsystem){
        this.mLimelightSubsystem = _mLimelightSubsystem;
        this.mFlywheelSubsystem = _mFlywheelSubsystem;
        this.mFlywheelPistonSubsystem = _mFlywheelPistonSubsystem;
        this.mDefaultFlywheelCommand = new DefaultFlywheelCommand(_mFlywheelSubsystem);
        this.mFullExtendFlywheelPistonCommand = new FullExtendFlywheelPistonCommand(mFlywheelPistonSubsystem);
        this.mFullRetractFlywheelPistonCommand = new FullRetractFlywheelPistonCommand(mFlywheelPistonSubsystem);

        addRequirements(_mFlywheelSubsystem, _mFlywheelPistonSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = DistanceMath.getDistY(mLimelightSubsystem.getVerticleOffset());
        if ( GREEN_START_ZONE <= distance && distance <= GREEN_END_ZONE ) {   // Green Zone (Back)
            mDefaultFlywheelCommand.setSpeedRPMs(GREEN_SPEED);        //needs to be changed!
            mFlywheelPistonSubsystem.setBottom(SolenoidValues.RETRACT);
            mFlywheelPistonSubsystem.setTop(SolenoidValues.RETRACT);
        } 
        else if ( YELLOW_START_ZONE <= distance && distance <= YELLOW_END_ZONE ) {   // Yellow Zone (Back)
            mDefaultFlywheelCommand.setSpeedRPMs(YELLOW_SPEED);
            mFlywheelPistonSubsystem.setBottom(SolenoidValues.RETRACT);
            mFlywheelPistonSubsystem.setTop(SolenoidValues.RETRACT);
        } 
        else if ( BLUE_START_ZONE <= distance && distance <= BLUE_END_ZONE ) {   // Blue Zone (Back)
            mDefaultFlywheelCommand.setSpeedRPMs(BLUE_SPEED);
            mFlywheelPistonSubsystem.setBottom(SolenoidValues.EXTEND);
            mFlywheelPistonSubsystem.setTop(SolenoidValues.EXTEND);
        } 
        else if ( RED_START_ZONE <= distance && distance <= RED_END_ZONE ) {   // Red Zone (Front)  :)
            mDefaultFlywheelCommand.setSpeedRPMs(RED_SPEED);
            mFlywheelPistonSubsystem.setBottom(SolenoidValues.EXTEND);
            mFlywheelPistonSubsystem.setTop(SolenoidValues.EXTEND);
        }
    }

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