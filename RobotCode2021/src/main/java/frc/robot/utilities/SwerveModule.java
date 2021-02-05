package frc.robot.utilities;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {

    private WPI_TalonFX driveFx;

    private WPI_TalonFX steerFx;

    private CANCoder steerEncoder;

    private static final Logger logger = Logger.getLogger(SwerveModule.class.getName());
    
    /**
     * Helper class for a swerve wheel. Holds two Falcon500's.
     * 
     * @param driveID  ID for the driving motor
     * @param turnID   ID for the turning motor
     */
    public SwerveModule(int driveID, int turnID) {
        driveFx = new WPI_TalonFX(driveID);
        steerFx = new WPI_TalonFX(turnID);
        steerEncoder = new CANCoder(turnID);
        //steerEncoder.

        //steerFx.configSelectedFeedbackSensor(RemoteFeedbackDevice);

        driveFx.configOpenloopRamp(2);
    }

    public void setAngle(double rotation) {
        if(RobotBase.isReal()){
            steerFx.set(ControlMode.Position, rotation);
        } else {
            steerFx.set(ControlMode.PercentOutput, rotation);
        }
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
}
