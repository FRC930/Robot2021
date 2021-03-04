/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.paths;

import edu.wpi.first.wpilibj.controller.RamseteController;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.commands.drivecommands.StopDriveCommand;
import frc.robot.commands.intakecommands.DeployIntakeCommand;
import frc.robot.commands.shootercommands.ShootPowerCellCommandGroup;
import frc.robot.commands.shootercommands.flywheelcommands.RunFlywheelAutoCommand;
import frc.robot.commands.turretcommads.AutoAimAutonomousCommand;
import frc.robot.commands.turretcommads.AutoTurretTurnCommand;

import java.util.List;

// -------- PATH DESCRIPTION -------- \\
// Mid Field - Move off intiation & Initial 3

public class FarmersBreakfastSkilletCommand extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  private final double AUTO_SHOOTER_SPEED = 0.8;
  public FarmersBreakfastSkilletCommand(DriveSubsystem dSubsystem,FlywheelSubsystem fSubsystem,IntakeMotorSubsystem iMotorSubsystem, IntakePistonSubsystem iPistonSubsystem,TurretSubsystem turSubsystem,LimelightSubsystem lLightSubsystem,TowerSubsystem towSubsystem,HopperSubsystem hSubsystem,KickerSubsystem kSubsystem) {
    
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.KSVOLTS,
            Constants.KVVOLT,
            Constants.KAVOLT),
            Constants.KDRIVEKINEMATICS,10);
    
    // Configurate the values of all trajectories for max velocity and acceleration
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.KMAXSPEED,
      Constants.KMAXACCELERATION)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.KDRIVEKINEMATICS)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    // -------- Trajectories -------- \\

    // Generates a trajectory 
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start 
        new Pose2d(inchesToMeters(0), inchesToMeters(0), new Rotation2d(0)),
        List.of(
            // Midpoints
        ),
        // End 5 feet infront of initiation line
        new Pose2d(inchesToMeters(60.0), inchesToMeters(0), new Rotation2d(0)),
        // Pass config
        config

    );

    // -------- RAMSETE Commands -------- \\
    // Creates a command that can be added to the command scheduler in the sequential command
    
    // Creates RAMSETE Command for first trajectory
    RamseteCommand ramseteCommand1 = new RamseteCommand(
        trajectory1,
        dSubsystem::getTankPose,
        new RamseteController(Constants.KRAMSETEB, Constants.KRAMSETEZETA),
        new SimpleMotorFeedforward(Constants.KSVOLTS,
                                   Constants.KVVOLT,
                                   Constants.KAVOLT),
        Constants.KDRIVEKINEMATICS,
        dSubsystem::getWheelSpeeds,
        new PIDController(Constants.KPDRIVEVEL, 0, 0),
        new PIDController(Constants.KPDRIVEVEL, 0, 0),
        // RamseteCommand passes volts to the callback
        dSubsystem::tankDriveVolts,
        dSubsystem
    );
    
    /*
    Path Description:
    -----------------
        Robot has 3 power cells set on top of the robot
        Robot Shoots 3 power cells and moves off initiation linE
    */

    addCommands(new RunFlywheelAutoCommand(fSubsystem, AUTO_SHOOTER_SPEED),
    new DeployIntakeCommand(iPistonSubsystem, iMotorSubsystem),
    new StopDriveCommand(dSubsystem),
    new AutoTurretTurnCommand(turSubsystem),
    new AutoAimAutonomousCommand(lLightSubsystem, turSubsystem, new PIDController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D)),
    new ParallelRaceGroup(new WaitCommand(2), new ShootPowerCellCommandGroup(towSubsystem, hSubsystem, kSubsystem)),
    ramseteCommand1
    );

  } // End of Constructor

  // Method to convert distances
  public double inchesToMeters(double inches) {
      double meters = inches / 39.37;
      return meters;
  }

} // End of Class