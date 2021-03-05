/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//DO NOT GO BACK TO ZERO AFTER LET GO
package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.SwerveModule;

//-------- SUBSYSTEM CLASS --------\\

public class SwerveDriveSubsystem extends SubsystemBase {

  // -------- DECLARATIONS --------\\
  private SwerveModule FRDrive;
  private SwerveModule BRDrive;
  private SwerveModule FLDrive;
  private SwerveModule BLDrive;
  
  private final PigeonIMU gyro;
  private boolean usingGyro;
  private boolean slowSpeed;
  private SwerveDriveOdometry od;
  private double gyroAngle;

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
  // -------- CONSTRUCTOR --------\\

  public SwerveDriveSubsystem(IntakeMotorSubsystem intake, boolean usingGyro, boolean slowSpeed) {
    setDriveMotors();
    gyro = new PigeonIMU(intake.getIntakeMotor());
    this.usingGyro = usingGyro;
    this.slowSpeed = slowSpeed;
    od = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
  }

  // -------- METHODS --------\\

  private void setDriveMotors() {

    // instantiates the swerve modules on the robot (We use 4)
    FRDrive = new SwerveModule(3, 7, 11);
    BRDrive = new SwerveModule(4, 8, 12);
    FLDrive = new SwerveModule(1, 5, 9);
    BLDrive = new SwerveModule(2, 6, 10);
  }

  /*// Assumption --  rotation: (-180 - 180) and gyroAngle: (-180 - 180)
  public double applyGyroAngle( double rotation, double gyroAngle) {
    boolean addAngle = false;
    // NOTE: May want to add instead gyroAngle
    double newRotation = rotation + ((addAngle?1:-1) * gyroAngle);
    if (newRotation > 180 )  {
      // example 225 -> -135
      newRotation = newRotation - 360;
    } else if (newRotation < -180 ) {
      // example -225 -> 135
      newRotation = newRotation + 360;
    }
    // Output needs to be between -180 and 180
    return newRotation;
  }*/

  /**
   * Sets each swerve module's angle and speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */

  public void drive(double targetX, double targetY, double rotation) {
    logger.entering(SwerveDriveSubsystem.class.getName(), "drive()");

    Rotation2d heading = Rotation2d.fromDegrees(gyro.getFusedHeading());
    double speedForward = targetY * Constants.KMAXSPEED;
    double speedStrafe = targetX * Constants.KMAXSPEED;
    double speedRotation = rotation * Constants.KMAXANGULARSPEED;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedForward, speedStrafe, speedRotation, heading);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.KMAXSPEED);

    FLDrive.drive(states[0]);
    FRDrive.drive(states[1]);
    BLDrive.drive(states[2]);
    BRDrive.drive(states[3]);
    
    logger.exiting(SwerveDriveSubsystem.class.getName(), "drive()");
  } // end of method drive()

  public void drive(SwerveModuleState[] states) {
    logger.entering(SwerveDriveSubsystem.class.getName(), "drive");

    FLDrive.drive(states[0]);
    FRDrive.drive(states[1]);
    BLDrive.drive(states[2]);
    BRDrive.drive(states[3]);
    logger.exiting(SwerveDriveSubsystem.class.getName(), "drive");
  } // end of method drive()

  public void stop(){
    FLDrive.setSpeed(0);
    FRDrive.setSpeed(0);
    BLDrive.setSpeed(0);
    BRDrive.setSpeed(0);
  }

  public void resetWheels(){
    FLDrive.setAngle(0);
    FRDrive.setAngle(0);
    BLDrive.setAngle(0);
    BRDrive.setAngle(0);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }
  
  public Pose2d getPose(){
    return od.getPoseMeters();
  }

  @Override
  public void periodic() {
    od.update(Rotation2d.fromDegrees(gyroAngle), FLDrive.getSwerveStates(), FRDrive.getSwerveStates(), BLDrive.getSwerveStates(), BRDrive.getSwerveStates());
  }
} // end of the class DriveSubsystem
