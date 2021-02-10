package frc.robot.utilities;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {

    private WPI_TalonFX driveFx;

    private WPI_TalonFX steerFx;

    private CANCoder steerEncoder;

    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    0.005,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            2 * Math.PI,
                            2 * Math.PI));

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

        driveFx.configOpenloopRamp(2);
    }

    public void setAngle(double rotation) {
        double turn = m_turningPIDController.calculate(steerEncoder.getAbsolutePosition(), rotation);
        steerFx.set(ControlMode.PercentOutput, turn);
    }

    public void setSpeed(double speed) {
        driveFx.set(ControlMode.PercentOutput, speed);
    }

    public void drive(double speed, double rotation) {
        setSpeed(speed);
        setAngle(rotation);
    }

    public double getAngle() {
        return steerFx.getSelectedSensorPosition();
    }

    public double getSpeed() {
        return driveFx.getSelectedSensorVelocity();
    }

    public double getClosedLoopError() {
        return steerFx.getClosedLoopError();
    }
}
