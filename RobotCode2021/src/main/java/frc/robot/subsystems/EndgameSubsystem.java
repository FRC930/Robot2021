package frc.robot.subsystems;

import java.util.logging.*;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndgameSubsystem extends SubsystemBase {
    
    private static final Logger logger = Logger.getLogger(EndgameSubsystem.class.getName());

    //private double speed;
    private final double maxSpeed = 1.0;
    private WPI_TalonFX endGameMotor;
    private double encoderPosition;
    private boolean limitHit = false;
    private boolean goingUp = true;;

    public EndgameSubsystem(int motorID) {
        endGameMotor = new WPI_TalonFX(motorID);
        
        if(RobotBase.isReal()){
            // just in case an encoder is added externally
        }

        logger.log(Constants.LOG_LEVEL_INFO, "Starting EndgameSubsystem");
    }

    // -------- METHODS --------\\

    public void setSpeed(double speed) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            encoderPosition = endGameMotor.getSelectedSensorPosition();
        }

        //encoderPosition = 0.0;

        //speed = clamp(speed); // speed cannot be negative due to the robot breaking if it goes in reverse

        endGameMotor.set(ControlMode.PercentOutput, speed);
        
        logger.log(Constants.LOG_LEVEL_FINER, "Set speed to " + getSpeed());
    }

    public double getSpeed() {
        return endGameMotor.getMotorOutputPercent();
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
            return endGameMotor.getSelectedSensorPosition();
        } else {
            return 0.0;
        }
    }

    public boolean getLimitState(){
        return limitHit;
    }

    public void setLimitState(boolean hit){
         limitHit = hit;
    }

    public boolean getUpState(){
        return goingUp;
    }

    public void setUpState(boolean up){
         goingUp = up;
    }

}
