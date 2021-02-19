package frc.robot.utilities;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {

    private WPI_TalonFX driveFx;

    private WPI_TalonFX steerFx;

    private CANCoder steerEncoder;

    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    0.25,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            6 * Math.PI,
                            6 * Math.PI));

    private static final Logger logger = Logger.getLogger(SwerveModule.class.getName());

    private boolean canDrive = false;
    
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

        if(false) {
        //SJW https://www.chiefdelphi.com/t/can-it-be-done-talonfx-cancoder-absolute-encoder/387576
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.remoteFilter0.remoteSensorDeviceID = steerEncoder.getDeviceID();
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        config.slot0.kP = 1.0;
        config.slot0.kI = 0.0;
        config.slot0.kD = 10.0;
        config.slot0.kF = 0.0;
        config.slot0.integralZone = 0;
        config.slot0.allowableClosedloopError = 0;
        config.motionAcceleration = 1000;
        config.motionCruiseVelocity = 100;
        driveFx.configAllSettings(config);
        }
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

        //Wait till azmuith
        // if(Math.abs(turn) > 0.08) {
        //     canDrive = false;
        // } else {
        //     canDrive = true;
        // }

        //SmartDashboard.putNumber("Speed"+driveFx.getDeviceID(), speed);
        SmartDashboard.putNumber("Rotation"+steerFx.getDeviceID(), rotation);
        //SmartDashboard.putNumber("Abs_Rotation"+steerFx.getDeviceID(), steerEncoder.getAbsolutePosition());

        logger.exiting(SwerveModule.class.getName(), "setAngle");
    }

    /**
    * Sets swerve module's  speed
    * 
    * @param speed The speed from -1 to 1
    */
    public void setSpeed(double speed) {
        //if(canDrive) {
            driveFx.set(ControlMode.PercentOutput, speed);
        //}
    }
    
    /**
    * Sets each swerve module's angle and speed
    * 
    * @param speed The speed from -1 to 1
    * @param rotation The Y position of the controller (Right stick)
    */
    public void drive(double speed, double rotation) {
        setSpeed(speed);
        setAngle(rotation);
    }

    /**
     * Sets swerve module's angle
     */
    public double getAngle() {
        return steerEncoder.getAbsolutePosition();
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