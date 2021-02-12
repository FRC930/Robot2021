/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.paths;

import edu.wpi.first.wpilibj.controller.RamseteController;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.commands.intakecommands.DeployIntakeCommand;
import frc.robot.commands.intakecommands.ReturnIntakeCommand;
import frc.robot.commands.shootercommands.ShootPowerCellCommandGroup;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import java.util.logging.*;

// -------- PATH DESCRIPTION -------- \\
// Alliance Side - Initial 3 & Trench 3 & Rendezvous 2

public class BarrelRacingCommand extends SequentialCommandGroup {

    private static final Logger logger = Logger.getLogger(BarrelRacingCommand.class.getName());
  /**
   * Path Description: ----------------- Shoot 3 from initiation line move through
   * trench to grab 3 balls Shoot 3 from trench position
   */
  public BarrelRacingCommand(DriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
      IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem tSubsystem,
      HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
      FlywheelPistonSubsystem fPistonSubsystem) {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.KSVOLTS, Constants.KVVOLT, Constants.KAVOLT), Constants.KDRIVEKINEMATICS,
        10);

    // Configurate the values of all trajectories for max velocity and acceleration
    TrajectoryConfig config = new TrajectoryConfig(Constants.KMAXSPEED, Constants.KMAXACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.KDRIVEKINEMATICS)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // -------- Trajectories -------- \\

    // Generates a trajectory for a path to move towards furthest ball in trench run
    String trajectoryJSON = "../../../../../../Resource/BarrelRacing.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        logger.log(Constants.LOG_LEVEL_INFO, "BarrelRacing tragectory path: " + trajectoryPath.toString());
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSON);
    }

    // -------- RAMSETE Commands -------- \\
    // Creates a command that can be added to the command scheduler in the
    // sequential command

    // Creates RAMSETE Command for first trajectory
    RamseteCommand ramseteCommand1 = new RamseteCommand(trajectory, dSubsystem::getPose,
        new RamseteController(Constants.KRAMSETEB, Constants.KRAMSETEZETA),
        new SimpleMotorFeedforward(Constants.KSVOLTS, Constants.KVVOLT, Constants.KAVOLT), Constants.KDRIVEKINEMATICS,
        dSubsystem::getWheelSpeeds, new PIDController(Constants.KPDRIVEVEL, 0, 0),
        new PIDController(Constants.KPDRIVEVEL, 0, 0),
        // RamseteCommand passes volts to the callback
        dSubsystem::tankDriveVolts, dSubsystem);

    /*
     * Path Description: ----------------- Shoot 3 from initiation line move through
     * trench to grab 3 balls Shoot 3 from trench position
     */

    addCommands(
        new ParallelRaceGroup(new WaitCommand(3),
            new ShootPowerCellCommandGroup(tSubsystem, hSubsystem, kSubsystem)), // Shoot 3 balls
        new ParallelRaceGroup(ramseteCommand1, new DeployIntakeCommand(iPistonSubsystem, iMotorSubsystem)), // Moving
                                                                                                            // trajectory
                                                                                                            // while
                                                                                                            // intaking
        new ReturnIntakeCommand(iPistonSubsystem, iMotorSubsystem), // Stop intaking
        new ParallelRaceGroup(new WaitCommand(3), new ShootPowerCellCommandGroup(tSubsystem, hSubsystem,
            kSubsystem))); // Shooting final 3 balls

  } // End of Constructor

  // Method to convert distances
  public double inchesToMeters(double inches) {
    double meters = inches / 39.37;
    logger.log(Constants.LOG_LEVEL_INFO, "meters: " + meters + "  inches: " + inches);
    return meters;
  }

} // End of class
