/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.towercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TowerSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h4>RunTowerCommand</h4> Runs the tower to move balls to the shooter.
 */
public class RunTowerCommand extends CommandBase {

    // -------- DECLARATIONS --------\\

    private TowerSubsystem towerSubsystem;

    // -------- CONSTRUCTOR --------\\
    /**
     * Creates instance of the command.
     * 
     * @param towerSubsystem instance of the TowerSubsystem
     */
    public RunTowerCommand(TowerSubsystem towerSubsystem) {
        this.towerSubsystem = towerSubsystem;
        addRequirements(towerSubsystem);
    }

    // -------- METHODS --------\\

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Runs the motor
        towerSubsystem.setSpeed(towerSubsystem.getTowerSpeed());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

} // end of class RunTowerCommand