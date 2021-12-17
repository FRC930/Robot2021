package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.AutonConfig;

public class DriveSubsystem extends SubsystemBase {
    public static final double KMAXSPEED = Units.inchesToMeters(16.2);
    public static final double KTRACKWIDTH = 0.5715;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(18.25);

    public static final int DRIVETRAIN_PIGEON_ID = 17;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(188.251);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(296.191);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(320.705);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(135.077);

    public static final double MAX_VOLTAGE = 12.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction()
            * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI / 10;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0) / 3;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private final PigeonIMU m_pigeon;

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final SwerveDriveOdometry swerveDriveOdometry;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DriveSubsystem(IntakeMotorSubsystem intake) {
        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(Mk3SwerveModuleHelper.GearRatio.FAST,
                FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(Mk3SwerveModuleHelper.GearRatio.FAST,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(Mk3SwerveModuleHelper.GearRatio.FAST,
                BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(Mk3SwerveModuleHelper.GearRatio.FAST,
                BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        swerveDriveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));

        m_pigeon = new PigeonIMU(intake.getIntakeMotor());

        AutonConfig.initInstance(this);
    }

    public void zeroGyro() {
        m_pigeon.setFusedHeading(0.0);
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void swerveDrive(SwerveModuleState[] states) {
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
    }

    public void rebootGyro() {
        m_pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return m_kinematics;
    }

    public void swerveStop() {
        drive(new ChassisSpeeds(0, 0, 0));
    }

    public Pose2d getPose() {
        Pose2d currentPose;
        if (swerveDriveOdometry != null) {
            currentPose = swerveDriveOdometry.getPoseMeters();
            return currentPose;
        } else {
            throw new RuntimeException("Tried to get pose with null swerve odometry");
        }
    }

    // public void swerveResetWheels() {
    // m_frontLeftModule.setAngle(0);
    // m_rightFrontModule.setAngle(0);
    // m_leftBackModule.setAngle(0);
    // m_rightBackModule.setAngle(0);
    // }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());

        if (swerveDriveOdometry != null) {
            swerveDriveOdometry.update(getGyroRotation(), states[0], states[1], states[2], states[3]);
        } else {
            throw new RuntimeException("Attempted to run the robot with a null swerve odometry");
        }
    }
}
