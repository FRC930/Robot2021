/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveMath;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\

public class SwerveDriveSubsystem extends SubsystemBase {

  // -------- DECLARATIONS --------\\
  private SwerveModule FRDrive;
  private SwerveModule BRDrive;
  private SwerveModule FLDrive;
  private SwerveModule BLDrive;

  private SwerveMath swerveMath;

  // -------- CONSTRUCTOR --------\\

  public SwerveDriveSubsystem() {
    setDriveMotors();
    swerveMath = new SwerveMath();
  }

  // -------- METHODS --------\\

  private void setDriveMotors() {

    // instantiates the swerve modules on the robot (We use 4)
    FRDrive = new SwerveModule(0, 1);
    BRDrive = new SwerveModule(2, 3);
    FLDrive = new SwerveModule(4, 5);
    BLDrive = new SwerveModule(6, 7);
  }

  /**
   * Sets each swerve module's angle and speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
  public void drive(double targetX, double targetY, double rotation) {
    FRDrive.drive(swerveMath.getFrontRightSpeed(targetX, targetY, rotation), swerveMath.getFrontRightAngle(targetX, targetY, rotation));
    BRDrive.drive(swerveMath.getBackRightSpeed(targetX, targetY, rotation), swerveMath.getBackRightAngle(targetX, targetY, rotation));
    FLDrive.drive(swerveMath.getFrontLeftSpeed(targetX, targetY, rotation), swerveMath.getFrontLeftAngle(targetX, targetY, rotation));
    BLDrive.drive(swerveMath.getBackLeftSpeed(targetX, targetY, rotation), swerveMath.getBackLeftAngle(targetX, targetY, rotation));
  } // end of method drive()

} // end of the class DriveSubsystem