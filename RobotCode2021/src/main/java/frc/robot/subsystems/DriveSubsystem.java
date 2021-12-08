/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

//import frc.robot.utilities.ShuffleboardUtility;
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
import frc.robot.utilities.AutonConfig;
import frc.robot.utilities.SwerveModule;

//-------- SUBSYSTEM CLASS --------\\

public class DriveSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\
    private static final Logger logger = Logger.getLogger(DriveSubsystem.class.getName());
    public static final double KSVOLTS = 0.67;
    public static final double KVVOLT = 2.31;
    public static final double KAVOLT = 0.0844; // this is in seconds squared per meter
    public static final double KMAXSPEED = Units.feetToMeters(16.2); //in meters per second 
    public static final double KMAXACCELERATION = 2; //in meters per seconds squared 
    public static final double KMAXANGULARSPEED = Math.PI * 2;
    //gyro values
    public static final double KRAMSETEB = 2;
    public static final double KRAMSETEZETA = 0.7;

    // Track width of our robot
    public static final double KTRACKWIDTH = 0.5715; // in meters
    public static final double KPDRIVEVEL = 1.49;
    // -------- DECLARATIONS --------\\

    private SwerveModule swerveRightFront;
    private SwerveModule swerveRightBack;
    private SwerveModule swerveLeftFront;
    private SwerveModule swerveLeftBack;

    // The gyro, used for autonomous
    private PigeonIMU gyro;

    private SwerveDriveOdometry swerveDriveOdometry;

    private int[] driveRightFrontIDs;
    private int[] driveLeftFrontIDs;
    private int[] driveRightBackIDs;
    private int[] driveLeftBackIDs;

    private final Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(10.625),
            Units.inchesToMeters(9.25));
    private final Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(10.625),
            Units.inchesToMeters(-9.25));
    private final Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-10.625),
            Units.inchesToMeters(9.25));
    private final Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-10.625),
            Units.inchesToMeters(-9.25));

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
            m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private double gyroAngle;

    private double speedModifierAuton = 1.0;
    private double speedModifierTeleop = 0.7;

    // Drive type enum
    public static enum DRIVE_TYPE {
        TANK_DRIVE(0), SWERVE_DRIVE(1);

        private int driveType;

        private DRIVE_TYPE(int _driveType) {
            driveType = _driveType;
        }

        public int Get() {
            return this.driveType;
        }
    }

    public static DRIVE_TYPE driveType;

    public static final double DRIVER_LEFT_AXIS_Y_DEADBAND = 0.15;
    public static final double DRIVER_LEFT_AXIS_X_DEADBAND = 0.15;
    public static final double DRIVER_RIGHT_AXIS_X_DEADBAND = 0.1;

    // -------- CONSTRUCTOR --------\\

    public DriveSubsystem(int[] additionalIDs, int[] _driveRightFrontIDs, int[] _driveLeftFrontIDs,
            int[] _driveRightBackIDs, int[] _driveLeftBackIDs, DRIVE_TYPE _driveType, IntakeMotorSubsystem _intake,
            boolean _usingGyro, boolean _slowSpeed) {

        driveType = _driveType;

        driveRightFrontIDs = _driveRightFrontIDs;
        driveLeftFrontIDs = _driveLeftFrontIDs;
        driveRightBackIDs = _driveRightBackIDs;
        driveLeftBackIDs = _driveLeftBackIDs;

        gyro = new PigeonIMU(_intake.getIntakeMotor());
        setSwerveDriveMotors();
        gyro = new PigeonIMU(_intake.getIntakeMotor());
        swerveDriveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        // initializing autonomous config. Done for performance reasons
        // AutonConfig.initInstance(this);
    }

    // -------- METHODS --------\\

    // SWERVE DRIVE
    public void setSwerveDriveMotors() {
        // instantiates the swerve modules on the robot (We use 4)
        swerveDriveOdometry = new SwerveDriveOdometry(m_kinematics, getAngleHeading(),
                new Pose2d(0, 0, new Rotation2d()));
        swerveRightFront = new SwerveModule(driveRightFrontIDs[0], driveRightFrontIDs[1], driveRightFrontIDs[2]);
        swerveRightBack = new SwerveModule(driveRightBackIDs[0], driveRightBackIDs[1], driveRightBackIDs[2]);
        swerveLeftFront = new SwerveModule(driveLeftFrontIDs[0], driveLeftFrontIDs[1], driveLeftFrontIDs[2]);
        swerveLeftBack = new SwerveModule(driveLeftBackIDs[0], driveLeftBackIDs[1], driveLeftBackIDs[2]);
    }

    // SWERVE DRIVE: Drive method
    public void swerveDrive(double targetX, double targetY, double rotation) {
        logger.entering(DriveSubsystem.class.getName(), "swerveDrive()");

        Rotation2d heading = Rotation2d.fromDegrees(gyro.getFusedHeading());
        double speedForward = (targetY * DriveSubsystem.KMAXSPEED);
        double speedStrafe = targetX * DriveSubsystem.KMAXSPEED;
        double speedRotation = rotation * DriveSubsystem.KMAXANGULARSPEED;

        // TODO: add in a proper flag for this
        // Create ChassisSpeeds to determine speed of robot frame
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedForward, speedStrafe, speedRotation, heading);
        // Turn off field centric ChassisSpeeds speeds = new ChassisSpeeds(speedForward,
        // speedStrafe, speedRotation);
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

        // Normalize speed so speed wont go over 1
        // This also will lower other wheel speeds if a speed goes over 1 on any wheel
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DriveSubsystem.KMAXSPEED);

        swerveLeftFront.drive(states[0], speedModifierTeleop);
        swerveRightFront.drive(states[1], speedModifierTeleop);
        swerveLeftBack.drive(states[2], speedModifierTeleop);
        swerveRightBack.drive(states[3], speedModifierTeleop);

        logger.exiting(DriveSubsystem.class.getName(), "swerveDrive()");
    } // end of method drive()

    public void swerveDrive(SwerveModuleState[] states) {
        logger.entering(DriveSubsystem.class.getName(), "swerveDrive()");

        swerveLeftFront.drive(states[0], speedModifierAuton);
        swerveRightFront.drive(states[1], speedModifierAuton);
        swerveLeftBack.drive(states[2], speedModifierAuton);
        swerveRightBack.drive(states[3], speedModifierAuton);

        logger.exiting(DriveSubsystem.class.getName(), "swerveDrive()");
    } // end of method swerveDrive()

    /**
     * Returns the gyro's yaw, in degrees
     * 
     * @return The gyro's yaw value, in degrees
     */
    public double getHeading() {
        // gyro.getYawPitchRoll(yawPitchRollValues);
        // return Math.IEEEremainder(yawPitchRollValues[0], 360);
        return gyro.getFusedHeading();
    }

    /**
     * Returns the current heading of the robot
     * 
     * @return angle
     */
    public Rotation2d getAngleHeading() {
        Rotation2d angle;
        angle = new Rotation2d(Math.toRadians(getHeading()));
        return angle;
    }

    /**
     * Reset the Swerve Odometry to "zero"
     */
    public void resetSwerveOdemetry() {
        swerveDriveOdometry.resetPosition(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), getAngleHeading());
    }

    /**
     * Reboot the PigeonIMU (This takes around 4-10 seconds)
     */
    public void rebootGyro() {
        gyro.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    // SWERVE DRIVE
    public Pose2d getSwervePose() {
        Pose2d VRtn = new Pose2d();
        if (swerveDriveOdometry != null) {
            VRtn = swerveDriveOdometry.getPoseMeters();
            System.out.println("Pose X: " + VRtn.getX() + " Pose Y: " + VRtn.getY());
        } else {
            // TODO: get LOG_LEVEL_ERROR
            logger.log(Constants.LOG_LEVEL_WARNING, "calling null swerveDriveOdometry");
            throw new RuntimeException("calling null swerveDriveOdometry");
        }
        return VRtn;
    }

    public Pose2d getPose() {
        return getSwervePose();
    }

    public void resetPose(Pose2d newPose) {
        swerveDriveOdometry.resetPosition(newPose, Rotation2d.fromDegrees(0));
    }

    // SWERVE DRIVE
    public void swerveStop() {
        swerveLeftFront.setSpeed(0);
        swerveRightFront.setSpeed(0);
        swerveLeftBack.setSpeed(0);
        swerveRightBack.setSpeed(0);
    }

    // SWERVE DRIVE
    public void swerveResetWheels() {
        swerveLeftFront.setAngle(0);
        swerveRightFront.setAngle(0);
        swerveLeftBack.setAngle(0);
        swerveRightBack.setAngle(0);
    }

    // SWERVE DRIVE
    public void setSpeedModifier(double newSpeedModifier) {
        speedModifierTeleop = newSpeedModifier;
    }

    // SWERVE DRIVE
    public SwerveDriveKinematics getSwerveKinematics() {
        return m_kinematics;
    }

    @Override
    // TODO: Figure out what the heck this does
    public void periodic() throws RuntimeException {
        // TODO: Set to different log level
        // logger.entering(DriveSubsystem.class.getName(), "periodic()");
        gyroAngle = gyro.getFusedHeading();
        if (swerveDriveOdometry != null) {
            swerveDriveOdometry.update(Rotation2d.fromDegrees(gyroAngle), swerveLeftFront.getSwerveStates(),
                    swerveRightFront.getSwerveStates(), swerveLeftBack.getSwerveStates(),
                    swerveRightBack.getSwerveStates());
        } else {
            // TODO: get LOG_LEVEL_ERROR
            logger.log(Constants.LOG_LEVEL_WARNING, "calling null <tank/swerve>DriveOdometry");
            throw new RuntimeException("calling null <tank/swerve>DriveOdometry");
        }
    }

} // end of the class DriveSubsystem