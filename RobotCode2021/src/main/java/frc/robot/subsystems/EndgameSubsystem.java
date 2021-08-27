package frc.robot.subsystems;
import java.util.logging.*;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls endgame motor
 * */ 
public class EndgameSubsystem extends SubsystemBase {
    
    private static final Logger logger = Logger.getLogger(EndgameSubsystem.class.getName());

    //Max Speed of Climber Arm
    private final double MAX_SPEED = 1.0;

    private WPI_TalonFX endGameMotor;
    //Starts limit hit at false
    private boolean limitHit = false;
    //Starts going up to true
    private boolean goingUp = true;;

    /**
     * instantiate the object
     * @param motorID
     */
    public EndgameSubsystem(int motorID) {
        endGameMotor = new WPI_TalonFX(motorID);

        logger.log(Constants.LOG_LEVEL_INFO, "Starting EndgameSubsystem");
    }

    // -------- METHODS --------\\

    /**
     * Sets speed of climber arm
     * @param speed
     */
    public void setSpeed(double speed) {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            encoderPosition = endGameMotor.getSelectedSensorPosition();
        }

        endGameMotor.set(ControlMode.PercentOutput, speed);
        
        logger.log(Constants.LOG_LEVEL_FINER, "Set speed to " + getSpeed());
    }

    /**
     * Gets speed of climber arm
     * @return
     */
    public double getSpeed() {
        return endGameMotor.getMotorOutputPercent();
    }

    /**
     * If ran on robot not in simulation returns encoder value
     * @return
     */
    public double getRawEncoderPosition() {
        // Dont use encoder in robot sim
        if(RobotBase.isReal()){
            return endGameMotor.getSelectedSensorPosition();
        } else {
            return 0.0;
        }
    }

    /**
     *  Gets limit state
     * @return
     */
    public boolean getLimitState(){
        return limitHit;
    }

    /**
     *  Sets Limit state
     * @param hit
     */
    public void setLimitState(boolean hit){
         limitHit = hit;
    }

    /**
     *  Gets up State
     * @return
     */
    public boolean getUpState(){
        return goingUp;
    }

    /**
     *  Sets Up State
     * @param up
     */
    public void setUpState(boolean up){
         goingUp = up;
    }

}
