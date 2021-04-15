package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

import java.util.logging.Logger;

import frc.robot.Constants;
//import frc.robot.utilities.ShuffleboardUtility;

//--------- SUBSYSTEM CLASS ---------\\

/**
 * <h3>FlywheelPistonSubsystem</h3> This class controls the piston on the
 * shooter
 */
public class FlywheelPistonSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    /**
     * This logger will be used for sending information to the user
     */
    private static final Logger logger = Logger.getLogger(FlywheelPistonSubsystem.class.getName());

    // -------- DECLARATIONS --------\\

    /**
     * This Solenoid will be used for changing the shooting angle
     */
    private Solenoid flywheelPistonTop;
    private Solenoid flywheelPistonBottom;

  //private ShuffleboardUtility shuffleboardUtility;

    /**
     * This enum will be used for choosing piston state
     */
    public enum SolenoidValues {

        EXTEND(true), RETRACT(false);

        private final boolean solenoidState;

        private SolenoidValues(boolean solenoidState) {
            this.solenoidState = solenoidState;
        }

        public boolean getSolenoidState() {
            return this.solenoidState;
        }
    }

    // -------- CONSTRUCTOR --------\\

    /**
     * This constructor will assign {@link #flywheelPiston} to the correct hardware
     */
    public FlywheelPistonSubsystem(int SHOOTER_SOLENOID_TOP_ID,int SHOOTER_SOLENOID_BOTTOM_ID) {
        //big
        flywheelPistonTop = new Solenoid(SHOOTER_SOLENOID_TOP_ID);
        //small
        flywheelPistonBottom = new Solenoid(SHOOTER_SOLENOID_BOTTOM_ID);
        //shuffleboardUtility = ShuffleboardUtility.getInstance();
    }

    // -------- METHODS --------\\

    /**
     * Sets the state of the piston
     * @param state the state of the piston
     */
    public void setTop(SolenoidValues state) {
        flywheelPistonTop.set(state.getSolenoidState());
        logger.log(Constants.LOG_LEVEL_FINE, "set(" + state + ")");

    }
    public void setBottom(SolenoidValues state) {
        flywheelPistonBottom.set(state.getSolenoidState());
        logger.log(Constants.LOG_LEVEL_FINE, "set(" + state + ")");

    }

    /**
     * This method will get the solenoid position
     * @return the solenoid position using the custom enum
     */
    public boolean getTop() {
        logger.log(Constants.LOG_LEVEL_FINER, "getPistonValue: " + (flywheelPistonTop.get() ? "True" : "False"));
        return flywheelPistonTop.get();
    }
    public boolean getBottom() {
        logger.log(Constants.LOG_LEVEL_FINER, "getPistonValue: " + (flywheelPistonBottom.get() ? "True" : "False"));
        return flywheelPistonBottom.get();
    }
} // end of class FlywheelPistonSubsystem