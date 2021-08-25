/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.hoppercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

//-------- COMMAND CLASS --------\\

/**
 * <h3>SetHopperCommand</h3>
 * 
 * SetHopperCommand controls the hopper used to store the power cells
 */
public class SetHopperCommand extends CommandBase {

    // -------- DECLARATIONS --------\\

    private HopperSubsystem m_HopperSubsystem;
    private double speed;
    private boolean isInverted;
    
    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>SetHopperCommand</h3>
     * 
     * This command runs once, setting the hopper speed to the passed value
     * 
     * @param hopperSubsystem the instance of
     *                        {@link frc.robot.subsystems.HopperSubsystem
     *                        HopperSubsystem} to use
     * @param speed           the speed at which to run the motor
     * @param isInverted      whether to run the motor in reverse or not
     */
    public SetHopperCommand(HopperSubsystem hopperSubsystem, double speed, boolean isInverted) {
        m_HopperSubsystem = hopperSubsystem;
        this.speed = speed;
        this.isInverted = isInverted;
        addRequirements(m_HopperSubsystem);
    }

    // -------- METHODS --------\\

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (isInverted) {
            m_HopperSubsystem.setSpeed(-speed);
        } else {
            m_HopperSubsystem.setSpeed(speed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
} // end of class DefaultHopperCommand