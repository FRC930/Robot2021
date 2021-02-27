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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  
  private final PigeonIMU gyro;

  private boolean slowSpeed;

  private final Translation2d m_frontLeftLocation = new Translation2d(
    Units.inchesToMeters(10.625),
    Units.inchesToMeters(9.25)
  );
  private final Translation2d m_frontRightLocation = new Translation2d(
    Units.inchesToMeters(10.625),
    Units.inchesToMeters(-9.25)
  );
  private final Translation2d m_backLeftLocation = new Translation2d(
    Units.inchesToMeters(-10.625),
    Units.inchesToMeters(9.25)
  );
  private final Translation2d m_backRightLocation = new Translation2d(
    Units.inchesToMeters(-10.625),
    Units.inchesToMeters(-9.25)
  );

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private static final Logger logger = Logger.getLogger(SwerveDriveSubsystem.class.getName());
/*
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          new Rotation2d(Units.degreesToRadians(gyro.getAbsoluteCompassHeading())),
          new Pose2d(),
          m_kinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
*/
  // -------- CONSTRUCTOR --------\\

  public SwerveDriveSubsystem(IntakeMotorSubsystem intake, boolean usingGyro, boolean slowSpeed) {
    setDriveMotors();
    gyro = new PigeonIMU(intake.getIntakeMotor());
    this.slowSpeed = slowSpeed;
    swerveMath = new SwerveMath(usingGyro);
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
    //
    //  gyro info in case we need it
    //double gyroAngle = -Units.degreesToRadians(gyro.getFusedHeading());
    double[] YahPitchRoll = {0.0, 0.0, 0.0};
    gyro.getYawPitchRoll(YahPitchRoll);
    double gyroAngle = -Units.degreesToRadians(YahPitchRoll[0]);
    System.out.println(gyroAngle);
    
    if(Math.abs(targetX) > Math.pow(0.1, 3) || Math.abs(targetY) > Math.pow(0.1, 3)){
      //
      //  calculate wheel positions
      //  -- grab needed outcomes and send to motors
      swerveMath.updateWheelPositions(targetX, targetY, rotation, gyroAngle);
      double FRAngle = swerveMath.getFrontRightAngle();
      double BRAngle = swerveMath.getBackRightAngle();
      double FLAngle = swerveMath.getFrontLeftAngle();
      double BLAngle = swerveMath.getBackLeftAngle();
      double FRSpeed = swerveMath.getFrontRightSpeed();
      double BRSpeed = swerveMath.getBackRightSpeed();
      double FLSpeed = swerveMath.getFrontLeftSpeed();
      double BLSpeed = swerveMath.getBackLeftSpeed();

      //
      //  scaling speed to be between -1 - 1
      double max = FRSpeed;
      if(BRSpeed > max) { max = BRSpeed; }
      if(FLSpeed > max) { max = FLSpeed; }
      if(BLSpeed > max) {max = BLSpeed; }

      if(max > 1) {
        FRSpeed /= max;
        BRSpeed /= max;
        FLSpeed /= max;
        BLSpeed /= max;
      }

      //System.out.println("BLAngle: " + BLAngle + " | FLAngle: " + FLAngle);
      //
      //  slow speed if need be.
      if(slowSpeed) {
        FRSpeed *= 0.5;
        BRSpeed *= 0.5;
        FLSpeed *= 0.5;
        BLSpeed *= 0.5;
      }

      SmartDashboard.putNumber("FRAngle: ", FRAngle); 
      SmartDashboard.putNumber("BRAngle: ", BRAngle); 
      SmartDashboard.putNumber("FLAngle: ", FLAngle); 
      SmartDashboard.putNumber("BLAngle: ", BLAngle);

      /*
      logger.log(Level.INFO, "FRAngle: " + FRDrive.getAngle());
      logger.log(Level.INFO, "BRAngle: " + swerveMath.getBackRightAngle(targetX, targetY, rotation));
      logger.log(Level.INFO, "FLAngle: " + swerveMath.getFrontLeftAngle(targetX, targetY, rotation));
      logger.log(Level.INFO, "BLAngle: " + swerveMath.getBackLeftAngle(targetX, targetY, rotation));
      */

      //logger.log(Level.INFO, "FR_CLE:" + FRDrive.getClosedLoopError());
      //logger.log(Level.INFO, "BR_CLE:" + BRDrive.getClosedLoopError());
      //logger.log(Level.INFO, "FL_CLE:" + FLDrive.getClosedLoopError());
      //logger.log(Level.INFO, "BL_CLE:" + BLDrive.getClosedLoopError());
      FRDrive.drive(FRSpeed, FRAngle);
      BRDrive.drive(BRSpeed, BRAngle);
      FLDrive.drive(FLSpeed, FLAngle);
      BLDrive.drive(BLSpeed, BLAngle);
    } else {
      FRDrive.drive(0.0, swerveMath.getFrontRightAngle());
      BRDrive.drive(0.0, swerveMath.getBackRightAngle());
      FLDrive.drive(0.0, swerveMath.getFrontLeftAngle());
      BLDrive.drive(0.0, swerveMath.getBackLeftAngle());
    }

    logger.exiting(SwerveDriveSubsystem.class.getName(), "drive");
  } // end of method drive()

} // end of the class DriveSubsystem