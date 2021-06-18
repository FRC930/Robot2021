package frc.robot.subsystems;

import java.util.logging.*;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndgameSubsystem extends SubsystemBase {
    
    private static final Logger logger = Logger.getLogger(EndgameSubsystem.class.getName());

    //private double speed;
    private final double maxSpeed = 1.0;
    private WPI_TalonSRX thanos;
    private DutyCycleEncoder encoder;
    private double encoderPosition;

    public EndgameSubsystem(int motorID, int encoderPortID) {
        thanos = new WPI_TalonSRX(motorID);
        
        if(RobotBase.isReal()){
            this.encoder = new DutyCycleEncoder(encoderPortID);
        }

        logger.log(Constants.LOG_LEVEL_INFO, "Starting EndgameSubsystem");
    }

    // -------- METHODS --------\\

    public void setSpeed(double speed) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            encoderPosition = encoder.get();
        }

        //encoderPosition = 0.0;

        speed = clamp(speed); // speed cannot be negative due to the robot breaking if it goes in reverse

        thanos.set(ControlMode.PercentOutput, speed);
        
        logger.log(Constants.LOG_LEVEL_FINER, "Set speed to " + getSpeed());
    }

    public double getSpeed() {
        return thanos.getMotorOutputPercent();
    }

    private double clamp(double speed) {
        if (speed > maxSpeed) {
            speed = maxSpeed;
        } else if (speed < 0) {//-maxSpeed) { // speed cannot be negative due to the robot breaking if it goes in reverse
            //speed = -maxSpeed;
            speed = 0;
        }
        return speed;
    }

    public double getRawEncoderPosition() {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            return this.encoder.get();
        } else {
            return 0.0;
        }
    }

}
