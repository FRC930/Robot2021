/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;

import frc.robot.commands.intakecommands.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import frc.robot.commands.shootercommands.ShootPowerCellCommandGroup;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.hoppercommands.SetAutonomousHopperCommand;
import frc.robot.commands.hoppercommands.SetHopperCommand;
import frc.robot.commands.turretcommads.AutoTurretTurnCommand;

import frc.robot.commands.drivecommands.StopDriveCommand;
import frc.robot.commands.turretcommads.AutoAimAutonomousCommand;
import frc.robot.commands.shootercommands.StopTowerKickerCommandGroup;
import frc.robot.commands.shootercommands.flywheelcommands.DefaultFlywheelCommand;
import frc.robot.commands.shootercommands.flywheelcommands.RunFlywheelAutoCommand;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import java.util.logging.*;

// -------- PATH DESCRIPTION -------- \\
// Alliance Side - Initial 3 & Trench 3 & Rendezvous 2

public class SaltAndPepperSkilletCommand extends SequentialCommandGroup {

    private static final Logger logger = Logger.getLogger(SaltAndPepperSkilletCommand.class.getName());
  /**
   * Path Description: ----------------- Shoot 3 from initiation line move through
   * trench to grab 3 balls Shoot 3 from trench position
   */
  public SaltAndPepperSkilletCommand(SwerveDriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
      IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem tSubsystem,
      HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
      FlywheelPistonSubsystem fPistonSubsystem) {

    // -------- Trajectories -------- \\

    // Generates a trajectory for a path to move towards furthest ball in trench run
    String trajectoryJSONpickupPath1 = Filesystem.getDeployDirectory() + "/Paths/pickupPath1.wpilib.json";
    Trajectory trajectorypickupPath1;
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONpickupPath1);
        logger.log(Constants.LOG_LEVEL_INFO, "pickupPath1 tragectory path: " + trajectoryPath.toString());
        trajectorypickupPath1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONpickupPath1, ex.getStackTrace());
        logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSONpickupPath1);
        throw new RuntimeException("Unable to open trajectory: " + trajectoryJSONpickupPath1);
    }

    String trajectoryJSONGoShoot5Meters = Filesystem.getDeployDirectory() + "/Paths/goShoot5Meeters.wpilib.json";
    Trajectory trajectoryGoShoot5Meters;
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONGoShoot5Meters);
        logger.log(Constants.LOG_LEVEL_INFO, "goShoot5Meeters tragectory path: " + trajectoryPath.toString());
        trajectoryGoShoot5Meters = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONGoShoot5Meters, ex.getStackTrace());
        logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSONGoShoot5Meters);
        throw new RuntimeException("Unable to open trajectory: " + trajectoryJSONGoShoot5Meters);
    }

    String trajectoryJSONPickUpPath2 = Filesystem.getDeployDirectory() + "/Paths/PickUpPath2Balls.wpilib.json";
    Trajectory trajectoryPickUpPath2;
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONPickUpPath2);
        logger.log(Constants.LOG_LEVEL_INFO, "PickUpPath2Balls tragectory path: " + trajectoryPath.toString());
        trajectoryPickUpPath2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONPickUpPath2, ex.getStackTrace());
        logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSONPickUpPath2);
        throw new RuntimeException("Unable to open trajectory: " + trajectoryJSONPickUpPath2);
    }

    String trajectoryJSONGoShoot = Filesystem.getDeployDirectory() + "/Paths/GoShoot.wpilib.json";
    Trajectory trajectoryGoShoot;
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONGoShoot);
        logger.log(Constants.LOG_LEVEL_INFO, "GoShoot tragectory path: " + trajectoryPath.toString());
        trajectoryGoShoot = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONGoShoot, ex.getStackTrace());
        logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSONGoShoot);
        throw new RuntimeException("Unable to open trajectory: " + trajectoryJSONGoShoot);
    }
// this is our config for how much power goes to the motors
var autoVoltageConstraint = new SwerveDriveKinematicsConstraint(dSubsystem.getKinematics(), Constants.KMAXSPEED);
//PID values
int kP = 1;
int kI = 0;
int kD = 0;
double maxV = Math.PI * 2;
double maxA = Math.PI;
// Configurate the values of all trajectories for max velocity and acceleration
TrajectoryConfig config =
new TrajectoryConfig(Constants.KMAXSPEED,
Constants.KMAXACCELERATION)
// Add kinematics to ensure max speed is actually obeyed
.setKinematics(dSubsystem.getKinematics())
.setEndVelocity(1)
// Apply the voltage constraint
.addConstraint(autoVoltageConstraint);

//a second trajectory config this one is reversed
TrajectoryConfig reverseConfig =
new TrajectoryConfig(Constants.KMAXSPEED,
Constants.KMAXACCELERATION)
// Add kinematics to ensure max speed is actually obeyed
.setKinematics(dSubsystem.getKinematics())
.setEndVelocity(1)
// Apply the voltage constraint
.addConstraint(autoVoltageConstraint)
.setReversed(true);

TrajectoryConfig slowConfig =
new TrajectoryConfig(Constants.KMAXSPEED,
2.0)
// Add kinematics to ensure max speed is actually obeyed
.setKinematics(dSubsystem.getKinematics())
// Apply the voltage constraint
.addConstraint(autoVoltageConstraint);


// -------- Trajectories -------- \\
// Generates a trajectory 

//this is our first trajectory
Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
    // Robot starts at X: 0 Y: 0 and a rotation of 0 
    new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
    List.of( 
        // Midpoints
    ),
    //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
    new Pose2d(inchesToMeters(120), inchesToMeters(-115), new Rotation2d(Math.toRadians(-65))), //X: was 130y is -135
    // Pass config
    config
);

// -------- RAMSETE Commands -------- \\
// Creates a command that can be added to the command scheduler in the sequential command
// The Ramsete Controller is a trajectory tracker that is built in to WPILib.
// This tracker can be used to accurately track trajectories with correction for minor disturbances.

// This is our first atuo command this will run the drivetrain using the first trajectory we made

SwerveControllerCommand command1 = new SwerveControllerCommand(trajectorypickupPath1, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);

SwerveControllerCommand command2 = new SwerveControllerCommand(trajectoryGoShoot5Meters, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);

SwerveControllerCommand command3 = new SwerveControllerCommand(trajectoryPickUpPath2, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);
   
SwerveControllerCommand command4 = new SwerveControllerCommand(trajectoryGoShoot, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);

// add commands here to run during auto
addCommands(
    // not sure why we had to do this; default command should do this
    //new RunFlywheelAutoCommand(fSubsystem, AUTO_SHOOTER_SPEED),
    //new DeployIntakeCommand(iPistonSubsystem, iMotorSubsystem),
    command1,
    //new StopDriveCommand(dSubsystem),
    command2,
    /*new StopDriveCommand(dSubsystem),
    new AutoTurretTurnCommand(turSubsystem),
    new AutoAimAutonomousCommand(lLightSubsystem, turSubsystem, new PIDController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D)),
    new ParallelRaceGroup(new WaitCommand(2), new ShootPowerCellCommandGroup(towSubsystem, hSubsystem, kSubsystem)),
    new RunFlywheelAutoCommand(fSubsystem, 0.85),
    new StopTowerKickerCommandGroup(towSubsystem, kSubsystem),
    new ParallelRaceGroup(command3, new SetAutonomousHopperCommand(hSubsystem)),
    new StopDriveCommand(dSubsystem),
    new AutoAimAutonomousCommand(lLightSubsystem, turSubsystem, new PIDController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D)),
    new ParallelRaceGroup(new WaitCommand(1.5), new ShootPowerCellCommandGroup(towSubsystem, hSubsystem, kSubsystem), new DefaultFlywheelCommand(fSubsystem, AUTO_SHOOTER_SPEED)), 
    new StopTowerKickerCommandGroup(towSubsystem, kSubsystem),
    new ReturnIntakeCommand(iPistonSubsystem, iMotorSubsystem),
    new RunFlywheelAutoCommand(fSubsystem, Constants.FLYWHEEL_TELEOP_SPEED),
    // not sure why we had to do this; default command should do this
    new SetHopperCommand(hSubsystem,0.0,false)*/
    command3, 
    command4
    );
    //returnIntakeCommand);

/*
Path Description:
-----------------
- Drive off intiation line
- Move to the side 2 Rendezvous Point balls
- Pick up two rendezvous point balls
- Shoot all 5 balls held
*/


//returnIntakeCommand);
      }

//converts our inches into meters
private double inchesToMeters(double inch){
return inch/39.3701;
} // End of class
      }

