package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {

    private AnalogInput ultrasonicSensor;

    public static enum GALACTIC_PATH {
        RED_PATH,
        BLUE_PATH
    }
    public static GALACTIC_PATH galacticPathType;

    public UltrasonicSubsystem(int analogPortID) {
        ultrasonicSensor = new AnalogInput(analogPortID);
    }

    public double getDistance() {
        return ultrasonicSensor.getValue() * 0.125;
        //return ultrasonicSensor_Left.getVoltage() * 0.125;
    }

    public void setGalacticPath(GALACTIC_PATH _galacticPathType) {
        galacticPathType = _galacticPathType;
    }

    public GALACTIC_PATH getGalacticPath() {
        return galacticPathType;
    }
}