package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class SwerveModule {

    private WPI_TalonFX driveFx;

    private WPI_TalonFX steerFx;

    public SwerveModule(int driveID, int turnID) {
        driveFx = new WPI_TalonFX(driveID);
        steerFx = new WPI_TalonFX(turnID);

        driveFx.configOpenloopRamp(2);
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
        return steerFx.getSelectedSensorPosition() / 6.86;
    }

    public double getSpeed() {
        return driveFx.getSelectedSensorVelocity() / 6.68;
    }
}
