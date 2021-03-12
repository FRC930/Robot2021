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
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;

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
import edu.wpi.first.wpiutil.math.VecBuilder;


//TODO: Organize all of the importants including these new imports due to merging of SwerveDriveSubsystem and DriveSubsystem
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

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
  private DifferentialDriveOdometry tankDriveOdometry;

  // The differential drive object itself
  private DifferentialDrive differentialDrive;

  //TODO: ORGANIZE SWERVE DRIVE VARIABLES
  private int[] driveRightFrontIDs;
  private int[] driveLeftFrontIDs;
  private int[] driveRightBackIDs;
  private int[] driveLeftBackIDs;

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
  private double prevX = 0;
  private double prevY = 0;

  private boolean usingGyro;

  private boolean slowSpeed;

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  private double gyroAngle;
  private SwerveDriveOdometry swerveDriveOdometry;

  private int gyroID;

  // tank drive
  private int shifterSolenoidID;

  // tank drive, auton config
  private final double MOTOR_RAMP_RATE = 0.75;//0.5;

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

  public DriveSubsystem(int[] additionalIDs, int[] _driveRightFrontIDs, int[] _driveLeftFrontIDs, int[] _driveRightBackIDs, int[] _driveLeftBackIDs, DRIVE_TYPE _driveType, IntakeMotorSubsystem _intake, boolean _usingGyro, boolean _slowSpeed) {
    
    driveType = _driveType;

    driveRightFrontIDs = _driveRightFrontIDs;
    driveLeftFrontIDs = _driveLeftFrontIDs;
    driveRightBackIDs = _driveRightBackIDs;
    driveLeftBackIDs = _driveLeftBackIDs;

    gyroID = additionalIDs[0];
    
    switch(driveType) {

      case TANK_DRIVE:
        shifterSolenoidID = additionalIDs[1];
        setTankDriveMotors();
        break;

      case SWERVE_DRIVE:
        setSwerveDriveMotors();
        //gyro = new PigeonIMU(_intake.getIntakeMotor());
        gyro = new PigeonIMU(gyroID);
        usingGyro = _usingGyro;
        slowSpeed = _slowSpeed;
        swerveDriveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        break;

      default: // default is error logging
        // TODO: find better error logging
        System.out.println("FAILURE TO SELECT DRIVE TYPE");
        break;

    }
  }

  //-------- METHODS --------\\

  // TANK DRIVE
  public void setTankDriveMotors() {
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
      gyroTalon = new WPI_TalonSRX(gyroID);
    } else {
      gyroTalon = new WPI_TalonSRX(gyroID + 10);
    }

    // the gyro attached to the talon, used to track position and rotation
    // TODO: Change this because GyroSubsystem already instantiates this!
    gyro = new PigeonIMU(gyroTalon);
  
    shifter = new Solenoid(shifterSolenoidID);

    //shuffleboardUtility = ShuffleboardUtility.getInstance();

    tankDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

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
    tankLeftFront.configOpenloopRamp(MOTOR_RAMP_RATE);
    tankRightFront.configOpenloopRamp(MOTOR_RAMP_RATE);
    // Sets up the differntial drive
    // drive = new DifferentialDrive(tankRightFront, tankLeftFront);
    shifter.set(true);
    //endgameClamp.set(true);
  }

  // SWERVE DRIVE
  public void setSwerveDriveMotors() {
    // instantiates the swerve modules on the robot (We use 4)
    swerveRightFront = new SwerveModule(driveRightFrontIDs[0], driveRightFrontIDs[1], driveRightFrontIDs[2]);
    swerveRightBack = new SwerveModule(driveRightBackIDs[0], driveRightBackIDs[1], driveRightBackIDs[2]);
    swerveLeftFront = new SwerveModule(driveLeftFrontIDs[0], driveLeftFrontIDs[1], driveLeftFrontIDs[2]);
    swerveLeftBack = new SwerveModule(driveLeftBackIDs[0], driveLeftBackIDs[1], driveLeftBackIDs[2]);
  }

  // TANK DRIVE
  // this is involved with shifting the gear for endgame
  public void setShifterState(boolean state) throws RuntimeException {
    if(shifter != null) {
      logger.log(Constants.LOG_LEVEL_FINER, "New shifter state: " + state);
      shifter.set(state);
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null shifter");
      throw new RuntimeException("calling null shifter");
    }
  }

  // TANK DRIVE
  // this is involved with the clampstate for endgame
  public void setEndgameClampState(boolean state) throws RuntimeException {
    if(endgameClamp != null) {
    logger.log(Constants.LOG_LEVEL_FINE, "Endgame clamp state: " + state);
    //endgameClamp.set(state);
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null endgameClamp");
      throw new RuntimeException("calling null endgameClamp");
    }
  }

  // TANK DRIVE: Drive method
  /**
   * Sets the left and right drivetrain motors to the speeds passed through the
   * parameters
   * 
   * @param leftSpeed  The speed of the left drivetrain motors
   * @param rightSpeed The speed of the right drivetrain motors
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    logger.entering(DriveSubsystem.class.getName(), "tankDrive()");

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

    logger.exiting(DriveSubsystem.class.getName(), "tankDrive()");
  } // end of method runAt()

  // SWERVE DRIVE: Drive method
  public void swerveDrive(double targetX, double targetY, double rotation) {
    logger.entering(DriveSubsystem.class.getName(), "swerveDrive()");

    Rotation2d heading = Rotation2d.fromDegrees(gyro.getFusedHeading());
    double speedForward = targetY * Constants.KMAXSPEED;
    double speedStrafe = targetX * Constants.KMAXSPEED;
    double speedRotation = rotation * Constants.KMAXANGULARSPEED;

    // Create ChassisSpeeds to determine speed of robot frame
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedForward, speedStrafe, speedRotation, heading);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize speed so speed wont go over 1
    //    This also will lower other wheel speeds if a speed goes over 1 on any wheel
    SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.KMAXSPEED);

    swerveLeftFront.drive(states[0]);
    swerveRightFront.drive(states[1]);
    swerveLeftBack.drive(states[2]);
    swerveRightBack.drive(states[3]);

    logger.exiting(DriveSubsystem.class.getName(), "swerveDrive()");
  } // end of method drive()

  public void swerveDrive(SwerveModuleState[] states) {
    logger.entering(SwerveDriveSubsystem.class.getName(), "swerveDrive()");

    swerveLeftFront.drive(states[0]);
    swerveRightFront.drive(states[1]);
    swerveLeftBack.drive(states[2]);
    swerveRightBack.drive(states[3]);
    
    logger.exiting(SwerveDriveSubsystem.class.getName(), "swerveDrive()");
  } // end of method swerveDrive()

  // TANK DRIVE
  /**
   * Returns the speed of the left drivetrain motors
   * 
   * @return The left drivetrain motor speed, ranging from -1 to 1
   * @throws Exception
   */
  public double getLeftSpeed() throws RuntimeException {
    double motorOutputPercent = 0;
    if(tankLeftFront != null) {
      motorOutputPercent = tankLeftFront.getMotorOutputPercent();
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null tankLeftFront");
      throw new RuntimeException("calling null tankLeftFront");
    }
    return motorOutputPercent;
  }

  // TANK DRIVE
  /**
   * Returns the speed of the right drivetrain motors
   * 
   * @return The right drivetrain motor speed, ranging from -1 to 1
   * @throws Exception
   */
  public double getRightSpeed() throws RuntimeException {
    double motorOutputPercent = 0;
    if(tankRightFront != null) {
      motorOutputPercent = tankRightFront.getMotorOutputPercent();
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null tankRightFront");
      throw new RuntimeException("calling null tankRightFront");
    }
    return motorOutputPercent;
  }

  // TANK DRIVE???
  /**
     * Returns the gyro's yaw, in degrees
     * 
     * @return The gyro's yaw value, in degrees
     */
  public double getHeading() {
    gyro.getYawPitchRoll(yawPitchRollValues);
    return Math.IEEEremainder(yawPitchRollValues[0], 360);
  }

  // TANK DRIVE
  /**
   * Returns the number of rotations from the left side of the drivetrain
   * 
   * @return A double, the # of rotations from the left drivetrain
   * @throws Exception
   */
  public double getLeftWheelRotations() throws RuntimeException {
    double leftWheelRotationsReturn = 0;
    if(tankLeftFront != null) {
      leftWheelRotationsReturn = tankLeftFront.getSelectedSensorPosition() * ((1.0 / 2048.0) * 0.152 * Math.PI) / DRIVE_GEAR_RATIO;
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null tankLeftFront");
      throw new RuntimeException("calling null tankLeftFront");
    }
    return leftWheelRotationsReturn;
  }

  // TANK DRIVE
  /**
   * Returns the number of rotations from the right side of the drivetrain
   * 
   * @return A double, the # of rotations from the right drivetrain
   * @throws Exception
   */
  public double getRightWheelRotations() throws RuntimeException {
    double rightWheelRotationsReturn = 0;
    if(tankLeftFront != null) {
      rightWheelRotationsReturn = tankRightFront.getSelectedSensorPosition() * ((1.0 / 2048.0) * 0.152 * Math.PI) / DRIVE_GEAR_RATIO;
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null tankRightFront");
      throw new RuntimeException("calling null tankRightFront");
    }
    return rightWheelRotationsReturn;
  }

  // TANK DRIVE
  // TODO: error handling
  /**
     * Returns the speeds in the differential drive
     * 
     * @return A DifferentialDriveWheelSpeeds object, has the rotations of the left and right wheels
     */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftWheelRotations(), getRightWheelRotations());
  }

  // TANK DRIVE
  /**
   * Resets the gyro
   * 
   * @throws Exception
   */
  public void resetOdometry(Pose2d pose) throws RuntimeException {
    if(tankDriveOdometry != null) {
      tankDriveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
      logger.log(Constants.LOG_LEVEL_FINE, "Odometer reset");
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null tankDriveOdometry");
      throw new RuntimeException("calling null tankDriveOdometry");
    }
  }

  // TANK DRIVE
  // TODO: Error creation and handling
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

  // TANK DRIVE
  // TODO: Error handling
  /**
     * Gets the left drivetrain encoder value
     * 
     * @return A double, of the getLeftWheelRotations()
     */
  public double getLeftEncoder() {
    return getLeftWheelRotations();
  }

  // TANK DRIVE
  // TODO: Error handling
  /**
     * Gets the right drivetrain encoder value
     * 
     * @return A double, of the getRightWheelRotations()
     */
  public double getRightEncoder() {
    return getRightWheelRotations();
  }

  // TANK DRIVE
  /**
     * Returns the position from the odometry
     * 
     * @return A Pose2d object, from the drive odometry
     */
  public Pose2d getTankPose() {
    Pose2d VRtn = new Pose2d();
    if(tankDriveOdometry != null) {
      VRtn = tankDriveOdometry.getPoseMeters();
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null tankDriveOdometry");
      throw new RuntimeException("calling null tankDriveOdometry");
    }
    return VRtn;
  }

  // SWERVE DRIVE
  public Pose2d getSwervePose() {
    Pose2d VRtn = new Pose2d();
    if(swerveDriveOdometry != null) {
      VRtn = swerveDriveOdometry.getPoseMeters();
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null swerveDriveOdometry");
      throw new RuntimeException("calling null swerveDriveOdometry");
    }
    return VRtn;
  }

  // SWERVE DRIVE
  public void swerveStop(){
    swerveLeftFront.setSpeed(0);
    swerveRightFront.setSpeed(0);
    swerveRightFront.setSpeed(0);
    swerveRightFront.setSpeed(0);
  }

  // SWERVE DRIVE
  public void swerveResetWheels(){
    swerveLeftFront.setAngle(0);
    swerveRightFront.setAngle(0);
    swerveLeftBack.setAngle(0);
    swerveRightBack.setAngle(0);
  }

  // SWERVE DRIVE
  public SwerveDriveKinematics swerveGetKinematics() {
    return m_kinematics;
  }

  // TANK DRIVE
  /**
   * Sets the max output the differential drive can give
   * 
   * @param maxOutput A double, specifying the maxiumum power that can be given by
   *                  the differential drive
   * @throws Exception
   */
  public void setMaxOutput(double maxOutput) throws RuntimeException {
    if(differentialDrive != null) {
      differentialDrive.setMaxOutput(maxOutput);
      logger.log(Constants.LOG_LEVEL_FINE, "New max output: " + maxOutput);
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null differentialDrive");
      throw new RuntimeException("calling null differentialDrive");
    }
  }

  @Override
  // TANK DRIVE
  // TODO: Figure out what the heck this does
  public void periodic() throws RuntimeException{
    //logger.entering(DriveSubsystem.class.getName(), "periodic()");

    if(tankDriveOdometry != null) {
      tankDriveOdometry.update(Rotation2d.fromDegrees(getHeading()), getLeftWheelRotations(), getRightWheelRotations());
    } else if (swerveDriveOdometry != null) {
      swerveDriveOdometry.update(Rotation2d.fromDegrees(gyroAngle), swerveLeftFront.getSwerveStates(), swerveRightFront.getSwerveStates(), swerveLeftBack.getSwerveStates(), swerveRightBack.getSwerveStates());
    } else {
      // TODO: get LOG_LEVEL_ERROR
      logger.log(Constants.LOG_LEVEL_WARNING, "calling null <tank/swerve>DriveOdometry");
      throw new RuntimeException("calling null <tank/swerve>DriveOdometry");
    }
    //shuffleboardUtility.setGyroYaw(getHeading());

    //logger.log(Constants.LOG_LEVEL_FINE, "Rotations: " + Rotation2d.fromDegrees(getHeading()) + "|| Left wheel rotations: " + getLeftWheelRotations() + "|| Right wheel rotations " + getRightWheelRotations());
    //logger.exiting(DriveSubsystem.class.getName(), "periodic()");  
  }

  // SWERVE DRIVE: helper method (no need for error creation and handling)
  // Assumption --  rotation: (-180 - 180) and gyroAngle: (-180 - 180)
  /*public double applyGyroAngle( double rotation, double gyroAngle) {
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

} // end of the class DriveSubsystem