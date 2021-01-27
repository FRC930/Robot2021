package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;


public class SwerveModule {

    private WPI_TalonFX driveFx;
    private CANCoder driveEncoder;

    private WPI_TalonFX steerFx;
    private CANCoder steerEncoder;

    public SwerveModule(int driveID, int turnID) {
        driveFx = new WPI_TalonFX(driveID);
        driveEncoder = new CANCoder(driveID);
        steerFx = new WPI_TalonFX(turnID);
        steerEncoder = new CANCoder(turnID);
    }

    public void setAngle(double rotation) {
        steerFx.set(ControlMode.Position, rotation);
    }

    public void setSpeed(double speed) {
        driveFx.set(ControlMode.PercentOutput, speed);
    }

    public void drive(double speed, double rotation) {
        setSpeed(speed);
        setAngle(rotation);
    }

    public double getAngle() {
        return steerEncoder.getPosition() / 6.86;
    }

    public double getSpeed() {
        return driveEncoder.getVelocity() / 6.68;
    }
}
