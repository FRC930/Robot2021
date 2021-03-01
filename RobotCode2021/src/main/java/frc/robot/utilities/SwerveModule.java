package frc.robot.utilities;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    private WPI_TalonFX driveFx;

    private WPI_TalonFX steerFx;

    private CANCoder steerEncoder;
    private double previousAngle = 0;

    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    0.25,
                    0.1,
                    0,
                    new TrapezoidProfile.Constraints(
                            6 * Math.PI,
                            6 * Math.PI));

    private static final Logger logger = Logger.getLogger(SwerveModule.class.getName());

    
    /**
     * Helper class for a swerve wheel. Holds two Falcon500's.
     * 
     * @param driveID  ID for the driving motor
     * @param turnID   ID for the turning motor
     */
    public SwerveModule(int driveID, int turnID, int encID) {
        driveFx = new WPI_TalonFX(driveID);
        steerFx = new WPI_TalonFX(turnID);
        steerEncoder = new CANCoder(encID);
        //steerEncoder.

        //steerFx.configSelectedFeedbackSensor(RemoteFeedbackDevice);

        //Set PID limits 
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveFx.configOpenloopRamp(2);
    }

    /**
    * Sets swerve module's angle 
    * 
    * @param rotation The rotation from -180 to 180
    */
    public void setAngle(double rotation) {
        logger.entering(SwerveModule.class.getName(), "setAngle");

        double turn = m_turningPIDController.calculate(Math.toRadians(steerEncoder.getAbsolutePosition()), Math.toRadians(rotation));
        //ShuffleboardTab tab = Shuffleboard.getTab("Tab 5");
        //tab.addNumber("Turn"+steerFx.getDeviceID(), () -> turn);
        //SmartDashboard.putNumber("Turn"+steerFx.getDeviceID(), turn);
        logger.log(Level.INFO, "SetSpeed: " + turn + " | AbsPos: " + steerEncoder.getAbsolutePosition() + " | Rotation: " + rotation);

        steerFx.set(ControlMode.PercentOutput, turn);

        //SmartDashboard.putNumber("Speed"+driveFx.getDeviceID(), speed);
        SmartDashboard.putNumber("Rotation"+steerFx.getDeviceID(), rotation);
        //SmartDashboard.putNumber("Error"+steerFx.getDeviceID(), m_turningPIDController.getPositionError());
        //SmartDashboard.putNumber("Abs_Rotation"+steerFx.getDeviceID(), steerEncoder.getAbsolutePosition());

        previousAngle = rotation;
        logger.exiting(SwerveModule.class.getName(), "setAngle");
    }

    /**
    * Sets swerve module's  speed
    * 
    * @param speed The speed from -1 to 1
    */
    public void setSpeed(double speed) {
        driveFx.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("Speed"+driveFx.getDeviceID(), speed);
    }

    public void drive(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        double angle = optimized.angle.getDegrees();
        double speed = optimized.speedMetersPerSecond;

        if (speed != 0) {
            setAngle(angle);
        } else {
            setAngle(previousAngle);
        }

        setSpeed(speed);
    }

    /**
     * Sets swerve module's angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    /**
     * Sets swerve module's speed
     */
    public double getSpeed() {
        return driveFx.getSelectedSensorVelocity();
    }

    /**
     * gets closed looperror
     */
    public double getClosedLoopError() {
        return steerFx.getClosedLoopError();
    }
}
