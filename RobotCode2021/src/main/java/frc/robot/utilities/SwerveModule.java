package frc.robot.utilities;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Controls swerve drive on robot
 */
public class SwerveModule {
    private WPI_TalonFX driveFx;

    private WPI_TalonFX steerFx;

    private CANCoder steerEncoder;

    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    0.25,
                    0.1,
                    0,
                    new TrapezoidProfile.Constraints(
                            6 * Math.PI,
                            6 * Math.PI));

    private static final Logger logger = Logger.getLogger(SwerveModule.class.getName());

    private final double RADIUS = 0.1016;

    private SimpleMotorFeedforward feedForward;

    private PIDController drivePID;
    
    /**
     * Helper class for a swerve wheel. Holds two Falcon500's.
     * 
     * @param driveID  ID for the driving motor
     * @param turnID   ID for the turning motor
     */
    public SwerveModule(int driveID, int turnID, int encID) {
        logger.entering(SwerveModule.class.getName(), "SwerveModule()");
        logger.log(Constants.LOG_LEVEL_FINER, "Creating SwerveModule...");
        logger.exiting(SwerveModule.class.getName(), "SwerveModule()");

        driveFx = new WPI_TalonFX(driveID);
        steerFx = new WPI_TalonFX(turnID);
        steerEncoder = new CANCoder(encID);

        // Force brake mode on the drive motors
        driveFx.setNeutralMode(NeutralMode.Brake);

        // Set PID limits 
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        drivePID = new PIDController(1.49, 0, 0);
        feedForward = new SimpleMotorFeedforward(0.67, 2.31, 0.0844);
    }

    /**
    * Sets swerve module's angle 
    * 
    * @param rotation The rotation from -180 to 180
    */
    public void setAngle(double rotation) {
        logger.entering(SwerveModule.class.getName(), "setAngle");

        double turn = m_turningPIDController.calculate(Math.toRadians(steerEncoder.getAbsolutePosition()), Math.toRadians(rotation));
        logger.log(Level.INFO, "SetSpeed: " + turn + " | AbsPos: " + steerEncoder.getAbsolutePosition() + " | Rotation: " + rotation);
        
        steerFx.set(ControlMode.PercentOutput, turn);

        SmartDashboard.putNumber("Rotation"+steerFx.getDeviceID(), rotation);

        logger.exiting(SwerveModule.class.getName(), "setAngle");
    }

    /**
    * Sets swerve module's  speed
    * 
    * @param speed The speed from -1 to 1
    */
    public void setSpeed(double speed) {
        logger.entering(SwerveModule.class.getName(), "setSpeed()");
        
        driveFx.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("Speed"+driveFx.getDeviceID(), speed);
        
        logger.log(Constants.LOG_LEVEL_FINER, "Set Wheel Speed to " + speed);
        logger.exiting(SwerveModule.class.getName(), "setSpeed()");
        //}
    }

    /**
    * Sets swerve module's  speed
    * 
    * @param state The SwerveModuleState this module should follow
    */
    public void drive(SwerveModuleState state, double speedModifier) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        double angle = optimized.angle.getDegrees();
        double speed = optimized.speedMetersPerSecond / DriveSubsystem.KMAXSPEED;

        if (speed != 0.0) {
            setAngle(angle);
        } else {
            setAngle(getAngle().getDegrees());
        }

        setSpeed(speed * speedModifier);
    }

    //gets the angle of wheel
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    //gets speed  of wheel
    public double getSpeed() {
        return ((driveFx.getSelectedSensorVelocity() * 10 / 2048) * RADIUS * Math.PI) / 6.86;
    }
    
    // Return the speed and angle of this module as a SwerveModuleState
    public SwerveModuleState getSwerveStates(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    /**
     * gets closed looperror
     */
    public double getClosedLoopError() {
        return steerFx.getClosedLoopError();
    }
}
