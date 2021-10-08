
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.shootercommands.pistoncommands.FullExtendFlywheelPistonCommand;
import frc.robot.commands.shootercommands.pistoncommands.FullRetractFlywheelPistonCommand;

//-------- COMMAND CLASS --------\\

public class AccuracyChallengeCommand extends SequentialCommandGroup {

    //-------- CONSTANTS --------\\
    //private final Logger logger = Logger.getLogger(this.getClass().getName());

    //Settings for zones.
    //Choose end of each desired shooting area in given shooting zone (green/yellow/blue/red).
    //Allows for motor speed optimization aswell (in rpms).

    public final static int GREEN_END_ZONE = 75;
    public final static double GREEN_SPEED = 3150.0;

    public final static int YELLOW_END_ZONE = 115;
    public final static double YELLOW_SPEED = 2270.0;

    public final static int BLUE_END_ZONE = 160;

    // original blue speed: 3150
    // 930's lakeshore speed
    public final static double BLUE_SPEED = 2900.0;

    //3950 RPM(long long shot) before Lakeshore champs
    //3500 went too far
    public final static double RED_SPEED = 3000.0;


    //-------- DECLARATIONS --------\\

    private FlywheelSubsystem mFlywheelSubsystem; 
    private FlywheelPistonSubsystem mFlywheelPistonSubsystem;
    private LimelightSubsystem mLimelightSubsystem;
    
    
    //-------- CONSTRUCTOR --------\\

    public AccuracyChallengeCommand(FlywheelSubsystem _mFlywheelSubsystem,
                                    FlywheelPistonSubsystem _mFlywheelPistonSubsystem,
                                    LimelightSubsystem _mLimelightSubsystem){
        this.mLimelightSubsystem = _mLimelightSubsystem;
        this.mFlywheelSubsystem = _mFlywheelSubsystem;
        this.mFlywheelPistonSubsystem = _mFlywheelPistonSubsystem;

        addRequirements(_mFlywheelSubsystem, _mFlywheelPistonSubsystem);
    }

    //-------- METHODS --------\\
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {   
        SmartDashboard.putString("Zone", "none");
    }

    // Code Below...
    //------------------------------------------------------
    // Checks if a the robot is in a given zone.
    // Sets the motor to the related speed.
    // Check around like 40 for config (Constants).
    //------------------------------------------------------

    // Called every time the scheduler runs while the command is scheduled.
    // TODO: Until we know what happens next season, we'll keep the setTop calls here
    @Override
    public void execute() {
         double distance = DistanceMath.getDistY(mLimelightSubsystem.getVerticleOffset());
         SmartDashboard.putNumber("Distance", distance);
         SmartDashboard.putNumber("Speed", mFlywheelSubsystem.getRadiansPerSecond());
         if (distance <= GREEN_END_ZONE ) {   // Green Zone (Back)
             mFlywheelSubsystem.setSpeedRPMs(GREEN_SPEED);        //needs to be changed!
             mFlywheelPistonSubsystem.setBottom(SolenoidValues.RETRACT);
             //mFlywheelPistonSubsystem.setTop(SolenoidValues.RETRACT);
             SmartDashboard.putString("Zone", "Green");
         } 
         else if ( GREEN_END_ZONE <= distance && distance <= YELLOW_END_ZONE ) {   // Yellow Zone (Back)
             mFlywheelSubsystem.setSpeedRPMs(YELLOW_SPEED);
             mFlywheelPistonSubsystem.setBottom(SolenoidValues.EXTEND);
             //mFlywheelPistonSubsystem.setTop(SolenoidValues.RETRACT);
             SmartDashboard.putString("Zone", "Yellow");
         } 
         else if ( YELLOW_END_ZONE <= distance && distance <= BLUE_END_ZONE ) {   // Blue Zone (Back)
             mFlywheelSubsystem.setSpeedRPMs(BLUE_SPEED);
             mFlywheelPistonSubsystem.setBottom(SolenoidValues.EXTEND);
             //mFlywheelPistonSubsystem.setTop(SolenoidValues.EXTEND);
             SmartDashboard.putString("Zone", "Blue");
         } 
         else if ( BLUE_END_ZONE <= distance) {   // Red Zone (Front)  :)
             mFlywheelSubsystem.setSpeedRPMs(RED_SPEED);
             mFlywheelPistonSubsystem.setBottom(SolenoidValues.EXTEND);
             //mFlywheelPistonSubsystem.setTop(SolenoidValues.EXTEND);
             SmartDashboard.putString("Zone", "Red");
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