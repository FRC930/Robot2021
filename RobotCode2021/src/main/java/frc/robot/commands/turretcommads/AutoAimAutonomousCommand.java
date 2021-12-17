/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.turretcommads;

import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//-------- PIDCOMMAND CLASS --------\\
/**
 * <h4>AutoAimAutonomousCommand</h4> Tracks the Power Port using the limelight
 * until it is with tolerances.
 */
public class AutoAimAutonomousCommand extends PIDCommand {

    // TODOS:
    // Change PID calculation comments inside the constructor
    // Move PID values from constants into subsystem
    // Move PID controller construction to this command

    // -------- DECLARATIONS --------\\

    private static final Logger logger = Logger.getLogger(AutoAimTurretCommand.class.getName());
    private LimelightSubsystem limelightSubsystem;
    private int counter;

    // -------- CONSTRUCTOR --------\\
    /**
     * Creates an instance of the class.
     * 
     * @param limelightSubsytem instantiated LimelightSubsystem
     * @param turretSubsystem   instantiated TurretSubsystem
     * @param controller        PIDController for the turret
     */
    public AutoAimAutonomousCommand(LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem,
            PIDController controller) {
        // Initial P = 0.065
        // Oscillations over time: 3 cycles per 1 second
        // Period = 0.33 : Took the oscillations over time and divided one by that
        // number
        // Calcualted P = 0.6 * Initial P = 0.039
        // Calculated I = (1.2 * Initial P) / Period = 0.24
        // Calculated D = (3 * Initial P * Period) / 40
        super(controller,
                // This lambda tells the controller where to get the input values from
                () -> {
                    SmartDashboard.putNumber("horiz off", limelightSubsystem.getHorizontalOffset());
                    // SmartDashboard.putNumber("PID error", value);
                    if (limelightSubsystem.getValidTargets()) {
                        return limelightSubsystem.getHorizontalOffset();
                    } else {
                        controller.reset();
                        return 0.0;
                    }
                },
                // The setpoint that the controller will try to acheive
                0.0,
                // This lambda tells the controller how to use the output
                (double output) -> {

                    // Make sure that we are not assigning some weird value to the motor
                    if (output > 1) {
                        output = 1;
                    } else if (output < -1) {
                        output = -1;
                    }
                    SmartDashboard.putNumber("controller", output);

                    if (Math.abs(limelightSubsystem.getHorizontalOffset()) < 27) {
                        turretSubsystem.setSpeed(output);

                    }
                },
                // Pass in the subsystems we will need
                turretSubsystem, limelightSubsystem); // End of super constructor

        logger.entering(AutoAimAutonomousCommand.class.getName(), "AutoAimTurretCommand");
        this.limelightSubsystem = limelightSubsystem;
        counter = 0;

        // Enable the controller to continuously get input
        this.getController().enableContinuousInput(-27, 27);

        // Set the tolerance of the controller
        this.getController().setTolerance(0.2);

        // Require the subsystems that we need
        addRequirements(limelightSubsystem, turretSubsystem);

    } // end of constructor AutoAimTurretCommand()

    @Override
    public void initialize() {
        // turn limelight LEDs on
        limelightSubsystem.setLightMode(limelightSubsystem.getLIMELIGHT_LEDS_ON());
    }

    @Override
    public boolean isFinished() {

        double offset = limelightSubsystem.getHorizontalOffset();
        boolean inRange = false;
        // isFinished() waits for 10 consecutive loops where the turret is aligned
        // within tolerances, then end the command.
        if (Math.abs(offset) < 1.5) {
            counter++;
            if (counter >= 10) {
                inRange = true;
            }
        } else {
            counter = 0;
            inRange = false;
        }

        return inRange;
    }

} // End DefaultTurretCommand class