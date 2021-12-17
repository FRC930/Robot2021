//-------- IMPORTS --------\\

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h3>IntakeMotorSubsystem</h3>
 * 
 * This class controls the intake motors
 */
public class IntakeMotorSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    /**
     * This is the logger we will use to communicate with the user
     */
    private static final Logger logger = Logger.getLogger(IntakeMotorSubsystem.class.getName());
    // Constant for our intake speed
    private final double INTAKE_SPEED = 0.6;

    // -------- DECLARATIONS --------\\

    /**
     * The motor controller that controls the intake motor
     */
    private WPI_TalonSRX intakeMotorController;

    // -------- CONSTRUCTOR --------\

    /**
     * This constructor initializes the {@link #intakeMotorController} to the proper
     * hardware
     * 
     * @param intakeID TalonSRX ID on the CAN Bus
     */
    public IntakeMotorSubsystem(int intakeID) {
        intakeMotorController = new WPI_TalonSRX(intakeID);
    }

    // -------- METHODS --------\\

    /**
     * <h3>setMotorSpeed</h3>
     * 
     * This method sets the intake motor speed to the passed variable
     * 
     * @param speed the speed at which to set the motor
     */
    public void setMotorSpeed(double speed) {
        System.out.print(intakeMotorController);
        intakeMotorController.set(ControlMode.PercentOutput, -speed);
        logger.log(Constants.LOG_LEVEL_FINE, "sets motor speed");
    }

    /**
     * <h3>getIntakeMotor</h3>
     * 
     * @return the motor controller object
     */
    public WPI_TalonSRX getIntakeMotor() {
        return intakeMotorController;
    }

    /**
     * <h3>getMotorSpeed</h3> This method returns the intake motor speed
     * 
     * @return the current motor speed
     */
    public double getMotorSpeed() {
        return intakeMotorController.getMotorOutputPercent();
    }

    /**
     * @return INTAKE_SPEED constant
     */
    public double getIntakeSpeed() {
        return INTAKE_SPEED;
    }

} // end of class IntakeMotorSubsystem
