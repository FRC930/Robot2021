/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//DO NOT GO BACK TO ZERO AFTER LET GO
package frc.robot.subsystems;

import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveMath;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

//-------- SUBSYSTEM CLASS --------\\

public class SwerveDriveSubsystem extends SubsystemBase {

  // -------- DECLARATIONS --------\\
  private SwerveModule FRDrive;
  private SwerveModule BRDrive;
  private SwerveModule FLDrive;
  private SwerveModule BLDrive;

  private SwerveMath swerveMath;

  private SwerveModuleState swerveModuleState;
  
  //private final PigeonIMU m_gyro = new PigeonIMU(0);

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  private double prevX = 0;
  private double prevY = 0;

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private static final Logger logger = Logger.getLogger(SwerveDriveSubsystem.class.getName());
/*
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          new Rotation2d(Units.degreesToRadians(m_gyro.getAbsoluteCompassHeading())),
          new Pose2d(),
          m_kinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
*/
  // -------- CONSTRUCTOR --------\\

  public SwerveDriveSubsystem() {
    setDriveMotors();
    swerveMath = new SwerveMath();
  }

  // -------- METHODS --------\\

  private void setDriveMotors() {

    // instantiates the swerve modules on the robot (We use 4)
    FRDrive = new SwerveModule(3, 7, 11);
    BRDrive = new SwerveModule(4, 8, 12);
    FLDrive = new SwerveModule(1, 5, 9);
    BLDrive = new SwerveModule(2, 6, 10);
  }

  /**
   * Sets each swerve module's angle and speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
  public void drive(double targetX, double targetY, double rotation) {
    logger.entering(SwerveDriveSubsystem.class.getName(), "drive");

    SmartDashboard.putNumber("FRAngle: ",swerveMath.getFrontRightAngle(targetX, targetY, rotation));
    SmartDashboard.putNumber("BRAngle: ", swerveMath.getBackRightAngle(targetX, targetY, rotation));
    SmartDashboard.putNumber("FLAngle: ",swerveMath.getFrontLeftAngle(targetX, targetY, rotation));
    SmartDashboard.putNumber("BLAngle: ",swerveMath.getBackLeftAngle(targetX, targetY, rotation));

    logger.log(Level.INFO, "FRAngle: " + FRDrive.getAngle());
    logger.log(Level.INFO, "BRAngle: " + swerveMath.getBackRightAngle(targetX, targetY, rotation));
    logger.log(Level.INFO, "FLAngle: " + swerveMath.getFrontLeftAngle(targetX, targetY, rotation));
    logger.log(Level.INFO, "BLAngle: " + swerveMath.getBackLeftAngle(targetX, targetY, rotation));

    //logger.log(Level.INFO, "FR_CLE:" + FRDrive.getClosedLoopError());
    //logger.log(Level.INFO, "BR_CLE:" + BRDrive.getClosedLoopError());
    //logger.log(Level.INFO, "FL_CLE:" + FLDrive.getClosedLoopError());
    //logger.log(Level.INFO, "BL_CLE:" + BLDrive.getClosedLoopError());

    
    if(Math.abs(targetX) > Math.pow(0.1, 3) || Math.abs(targetY) > Math.pow(0.1, 3)){
      prevX = targetX;
      prevY = targetY;
      FRDrive.drive(swerveMath.getFrontRightSpeed(targetX, targetY, rotation), swerveMath.getFrontRightAngle(targetX, targetY, rotation));
      BRDrive.drive(swerveMath.getBackRightSpeed(targetX, targetY, rotation), swerveMath.getBackRightAngle(targetX, targetY, rotation));
      FLDrive.drive(swerveMath.getFrontLeftSpeed(targetX, targetY, rotation), swerveMath.getFrontLeftAngle(targetX, targetY, rotation));
      BLDrive.drive(swerveMath.getBackLeftSpeed(targetX, targetY, rotation), swerveMath.getBackLeftAngle(targetX, targetY, rotation));
    } else {
      FRDrive.drive(swerveMath.getFrontRightSpeed(targetX, targetY, rotation), swerveMath.getFrontRightAngle(prevX, prevY, rotation));
      BRDrive.drive(swerveMath.getBackRightSpeed(targetX, targetY, rotation), swerveMath.getBackRightAngle(prevX, prevY, rotation));
      FLDrive.drive(swerveMath.getFrontLeftSpeed(targetX, targetY, rotation), swerveMath.getFrontLeftAngle(prevX, prevY, rotation));
      BLDrive.drive(swerveMath.getBackLeftSpeed(targetX, targetY, rotation), swerveMath.getBackLeftAngle(prevX, prevY, rotation));
    }

    logger.exiting(SwerveDriveSubsystem.class.getName(), "drive");
  } // end of method drive()

} // end of the class DriveSubsystem