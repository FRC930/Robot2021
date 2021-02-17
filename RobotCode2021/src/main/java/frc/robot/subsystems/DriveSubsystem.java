/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveMath;

import frc.robot.Constants;
//import frc.robot.utilities.ShuffleboardUtility;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//-------- SUBSYSTEM CLASS --------\\

public class DriveSubsystem extends SubsystemBase {

  // -------- CONSTANTS --------\\
  private final double DRIVE_GEAR_RATIO = 9.88;
  private static final Logger logger = Logger.getLogger(DriveSubsystem.class.getName());

  // -------- DECLARATIONS --------\\
  private WPI_TalonFX tankRightFront;
  private WPI_TalonFX tankRightBack;
  private WPI_TalonFX tankLeftFront;
  private WPI_TalonFX tankLeftBack;
  //private ShuffleboardUtility shuffleboardUtility;

  private SwerveModule swerveRightFront;
  private SwerveModule swerveRightBack;
  private SwerveModule swerveLeftFront;
  private SwerveModule swerveLeftBack;

  // The intake talon motor controller, has the gyro attached to it
  private WPI_TalonSRX gyroTalon;

  // The gyro, used for autonomous
  private PigeonIMU gyro;

  private Solenoid shifter;
  private Solenoid endgameClamp;

  // Values, used to store the yaw, pitch, and roll (the robot's rotation)
  private double yawPitchRollValues[] = new double[3];

  // The odometry of the differential drive
  private DifferentialDriveOdometry driveOdometry;

  // The differential drive object itself
  private DifferentialDrive differentialDrive;

  private int[] driveRightFrontIDs;
  private int[] driveLeftFrontIDs;
  private int[] driveRightBackIDs;
  private int[] driveLeftBackIDs;

  //Drive type enum
  public static enum DRIVE_TYPE {
    TANK_DRIVE(0),
    SWERVE_DRIVE(1);

    private int driveType;

    private DRIVE_TYPE(int _driveType) {
        driveType = _driveType;
    }
    public int Get() {
        return this.driveType;
    }
  }
  public static DRIVE_TYPE driveType;

  // -------- CONSTRUCTOR --------\\

  public DriveSubsystem() {
    setDriveMotors();
  }

  public DriveSubsystem(int[] _driveRightFrontIDs, int[] _driveLeftFrontIDs, int[] _driveRightBackIDs, int[] _driveLeftBackIDs, int _gyroID, DRIVE_TYPE _driveType) {
    
    driveType = _driveType;

    driveRightFrontIDs = _driveRightFrontIDs;
    driveLeftFrontIDs = _driveLeftFrontIDs;
    driveRightBackIDs = _driveRightBackIDs;
    driveLeftBackIDs = _driveLeftBackIDs;
    
    switch(driveType) {
      case TANK_DRIVE:
        setDriveMotors();
        break;
      case SWERVE_DRIVE:
        setDriveMotors();
        break;
      default: // default is error logging
        // TODO: find better error logging
        System.out.println("FAILURE TO SELECT DRIVE TYPE");
        break;
    }
  }

  //-------- METHODS --------\\

  private void setDriveMotors() {

    // instantiates the drive motors
    tankRightFront = new WPI_TalonFX(driveRightFrontIDs[0]);
    tankRightBack = new WPI_TalonFX(driveRightBackIDs[0]);
    tankLeftFront = new WPI_TalonFX(driveLeftFrontIDs[0]);
    tankLeftBack = new WPI_TalonFX(driveLeftBackIDs[0]);

    // the talon that controls intake, used to get the piston
    // TODO: Change this because IntakeSubsystem already instantiates this!
    // Move gyro to port 16 so simulator does not break
    if(RobotBase.isReal())
    {
      gyroTalon = new WPI_TalonSRX(Constants.INTAKE_ID);
    } else {
      gyroTalon = new WPI_TalonSRX(Constants.INTAKE_ID + 10);
    }

    // the gyro attached to the talon, used to track position and rotation
    // TODO: Change this because GyroSubsystem already instantiates this!
    gyro = new PigeonIMU(gyroTalon);
   
    shifter = new Solenoid(Constants.SHIFTER_SOLENOID_ID);

    //shuffleboardUtility = ShuffleboardUtility.getInstance();

    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // Inverts the direction of the drive motors
    tankLeftFront.setInverted(true);
    tankLeftBack.setInverted(true);
    tankRightFront.setInverted(false);
    tankRightBack.setInverted(false);

    // Resets drive motor encoders to 0
    tankLeftFront.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
    tankRightFront.getSensorCollection().setIntegratedSensorPosition(0.0, 100);

    tankLeftFront.setSensorPhase(true);
    tankLeftBack.setSensorPhase(true);
    tankRightFront.setSensorPhase(false);
    tankRightBack.setSensorPhase(false);

    // Mirror primary motor controllers on each side (Not when in simulation)
    if(RobotBase.isReal()){
      tankLeftBack.follow(tankLeftFront);
      tankRightBack.follow(tankRightFront);
    }
    
    // Sets the ramp rate of the robot, this will need to be configued
    tankLeftFront.configOpenloopRamp(Constants.MOTOR_RAMP_RATE);
    tankRightFront.configOpenloopRamp(Constants.MOTOR_RAMP_RATE);
    // Sets up the differntial drive
    // drive = new DifferentialDrive(tankRightFront, tankLeftFront);
    shifter.set(true);
    //endgameClamp.set(true);
  }

  public void setShifterState(boolean state) {
    logger.log(Constants.LOG_LEVEL_FINE, "New shifter state: " + state);
    shifter.set(state);
  }

  public void setEndgameClampState(boolean state) {
    logger.log(Constants.LOG_LEVEL_FINE, "Endgame clamp state: " + state);
    //endgameClamp.set(state);
  }

  /**
   * Sets the left and right drivetrain motors to the speeds passed through the
   * parameters
   * 
   * @param leftSpeed  The speed of the left drivetrain motors
   * @param rightSpeed The speed of the right drivetrain motors
   */
  public void runAt(double leftSpeed, double rightSpeed) {
    logger.entering(DriveSubsystem.class.getName(), "runAt()");

    //TODO: We should be seeing this on the Shuffleboard
    logger.log(Constants.LOG_LEVEL_FINE, "New left speed: " + leftSpeed + "|| New right speed: " + rightSpeed);
    logger.log(Constants.LOG_LEVEL_FINE, "running " + "left encoder " + getLeftWheelRotations() + " | right encoder " + getRightWheelRotations());

    if(!shifter.get()) {
      leftSpeed *= 0.3;
      rightSpeed *= 0.3;
    }
    
    tankLeftFront.set(leftSpeed);
    tankRightFront.set(rightSpeed);
    
    // If we are simulating robot set motor speeds manually
    if(!RobotBase.isReal()){
      tankLeftBack.set(leftSpeed);
      tankRightBack.set(rightSpeed);
    }

    logger.exiting(DriveSubsystem.class.getName(), "runAt()");
  } // end of method runAt()

  /**
     * Returns the speed of the left drivetrain motors
     * 
     * @return The left drivetrain motor speed, ranging from -1 to 1
     */
  public double getLeftSpeed() {
    return tankLeftFront.getMotorOutputPercent();
  }

  /**
     * Returns the speed of the right drivetrain motors
     * 
     * @return The right drivetrain motor speed, ranging from -1 to 1
     */
  public double getRightSpeed() {
    return tankRightFront.getMotorOutputPercent();
  }

  /**
     * Returns the gyro's yaw, in degrees
     * 
     * @return The gyro's yaw value, in degrees
     */
  public double getHeading() {
    gyro.getYawPitchRoll(yawPitchRollValues);
    return Math.IEEEremainder(yawPitchRollValues[0], 360);
  }

  /**
     * Returns the number of rotations from the left side of the drivetrain
     * 
     * @return A double, the # of rotations from the left drivetrain
     */
  public double getLeftWheelRotations() {
    return tankLeftFront.getSelectedSensorPosition() * ((1.0 / 2048.0) * 0.152 * Math.PI) / DRIVE_GEAR_RATIO;
  }

  /**
     * Returns the number of rotations from the right side of the drivetrain
     * 
     * @return A double, the # of rotations from the right drivetrain
     */
  public double getRightWheelRotations() {
      return tankRightFront.getSelectedSensorPosition() * ((1.0 / 2048.0) * 0.152 * Math.PI) / DRIVE_GEAR_RATIO;
  }

  /**
     * Returns the speeds in the differential drive
     * 
     * @return A DifferentialDriveWheelSpeeds object, has the rotations of the left and right wheels
     */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftWheelRotations(), getRightWheelRotations());
  }

  /**
     * Resets the gyro
     */
  public void resetOdometry(Pose2d pose) {
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    logger.log(Constants.LOG_LEVEL_FINE, "Odometer reset");
  }

  /**
     * Runs the drivetrain motors, but by sending the voltage amount instead  
     */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    logger.entering(DriveSubsystem.class.getName(), "tankDriveVolts()");

    logger.log(Constants.LOG_LEVEL_FINE, "Drivetrain Moving: " + leftVolts + " " + rightVolts);

    //TODO: Change for Prac Robot
    // tankRightFront.setVoltage(rightVolts);
    // tankLeftFront.setVoltage(-leftVolts);

    //TODO: Change for Comp Robot
    tankRightFront.setVoltage(rightVolts);
    tankLeftFront.setVoltage(leftVolts);

    logger.exiting(DriveSubsystem.class.getName(), "tankDriveVolts()");
  } // end of method tankDriveVolts()

  //-- TODO: Update getLeftEncoder() and getRightEncoder()

  /**
     * Gets the left drivetrain encoder value
     * 
     * @return A double, of the getLeftWheelRotations()
     */
  public double getLeftEncoder() {
    return getLeftWheelRotations();
  }

  /**
     * Gets the right drivetrain encoder value
     * 
     * @return A double, of the getRightWheelRotations()
     */
  public double getRightEncoder() {
    return getRightWheelRotations();
  }

  /**
     * Returns the position from the odometry
     * 
     * @return A Pose2d object, from the drive odometry
     */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  /**
     * Sets the max output the differential drive can give
     * 
     * @param maxOutput A double, specifying the maxiumum power that can be given by the differential drive
     */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
    logger.log(Constants.LOG_LEVEL_FINE, "New max output: " + maxOutput);
  }

  @Override
  public void periodic() {
    //logger.entering(DriveSubsystem.class.getName(), "periodic()");

    driveOdometry.update(Rotation2d.fromDegrees(getHeading()), getLeftWheelRotations(), getRightWheelRotations());
    //shuffleboardUtility.setGyroYaw(getHeading());

    //logger.log(Constants.LOG_LEVEL_FINE, "Rotations: " + Rotation2d.fromDegrees(getHeading()) + "|| Left wheel rotations: " + getLeftWheelRotations() + "|| Right wheel rotations " + getRightWheelRotations());
    //logger.exiting(DriveSubsystem.class.getName(), "periodic()");  
  }

} // end of the class DriveSubsystem