/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.LakeshorePath;

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

import frc.robot.subsystems.FlywheelPistonSubsystem;

import frc.robot.commands.intakecommands.*;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import java.util.ArrayList;
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
import frc.robot.utilities.AutonConfig;
import frc.robot.commands.hoppercommands.SetAutonomousHopperCommand;
import frc.robot.commands.turretcommads.AutoTurretTurnCommand;

import frc.robot.commands.drivecommands.StopDriveCommand;
import frc.robot.commands.turretcommads.AutoAimAutonomousCommand;
import frc.robot.commands.shootercommands.StopTowerKickerCommandGroup;
import frc.robot.commands.shootercommands.flywheelcommands.DefaultFlywheelCommand;
import frc.robot.commands.shootercommands.flywheelcommands.RunFlywheelAutoCommand;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import java.util.logging.*;

// -------- PATH DESCRIPTION -------- \\
// Alliance Side - Initial 3 & Trench 3 & Rendezvous 2

public class LeFisheThe2nd extends SequentialCommandGroup {

    private static final Logger logger = Logger.getLogger(LeFishe.class.getName());
    private Trajectory trajectory1;
    private    Trajectory trajectory2;
    private    Trajectory trajectory3;
    private    Trajectory trajectory4;
    private double xOffset = inchesToMeters(35.25);
    private double yOffset = inchesToMeters(6.5);
  /**
   * Path Description: ----------------- Shoot 3 from initiation line move through
   * trench to grab 3 balls Shoot 3 from trench position
   */
  public LeFisheThe2nd(DriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
      IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem tSubsystem,
      HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
      FlywheelPistonSubsystem fPistonSubsystem) {

    // -------- Trajectories -------- \\

    // Generates a trajectory for a path to move towards furthest ball in trench run
    trajectory1 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(3.195, -1.857, new Rotation2d(Math.toRadians(28))),
             List.of( 
                 new Translation2d(5.165 + xOffset, -1.018 + yOffset)
             ),
             new Pose2d(8.009 + xOffset, -0.538 - yOffset, new Rotation2d(Math.toRadians(-62))),
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory2 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                 new Pose2d(8.009, -0.538, new Rotation2d(Math.toRadians(28))),
                 List.of( 
                     new Translation2d(5.199 + xOffset, -2.217 + yOffset)
                 ),
                 new Pose2d(6.398 + xOffset, -3.228 - yOffset, new Rotation2d(Math.toRadians(-62))),
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
                );
    




// this is our config for how much power goes to the motors
//var autoVoltageConstraint = new SwerveDriveKinematicsConstraint(dSubsystem.getSwerveKinematics(), Constants.KMAXSPEED);
//PID values
double kPX = /*1.1*/ 0;
        double kIX = 0;
        double kDX = 0;
        double kPY = /*2*/ 0;
        double kIY = 0;
        double kDY = 0;
        double kPRot = 3;
        double kIRot = 0;
        double kDRot = 0;
double maxV = Math.PI * 2;
double maxA = Math.PI;
// Configurate the values of all trajectories for max velocity and acceleration

// -------- RAMSETE Commands -------- \\
// Creates a command that can be added to the command scheduler in the sequential command
// The Ramsete Controller is a trajectory tracker that is built in to WPILib.
// This tracker can be used to accurately track trajectories with correction for minor disturbances.

// This is our first atuo command this will run the drivetrain using the first trajectory we made

SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory1, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
    new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
    new TrapezoidProfile.Constraints(maxV, maxA)), () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

    SwerveControllerCommand command2 = new SwerveControllerCommand(trajectory2, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
    new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
    new TrapezoidProfile.Constraints(maxV, maxA)), () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

    SwerveControllerCommand command3 = new SwerveControllerCommand(trajectory3, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
    new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
    new TrapezoidProfile.Constraints(maxV, maxA)), () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

    SwerveControllerCommand command4 = new SwerveControllerCommand(trajectory4, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
    new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
    new TrapezoidProfile.Constraints(maxV, maxA)), () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

    RunIntakeMotorsCommand rollerCommand = new RunIntakeMotorsCommand(iMotorSubsystem);



/*
Path Description:
-----------------
- Drive off intiation line
- Move to the side 2 Rendezvous Point balls
- Pick up two rendezvous point balls
- Shoot all 5 balls held
*/
    Pose2d finalPose = trajectory1.getStates().get(trajectory1.getStates().size()-1).poseMeters;
    System.out.println("*******First Robot Pose: " + dSubsystem.getPose() + "********");
    System.out.println("*******Initial Path Pose: "+ trajectory1.getInitialPose() + " ********");
    //dSubsystem.resetPose(trajectory1.getInitialPose());
    System.out.println("*******Adjusted First Robot Pose: " + dSubsystem.getPose() + "********");
    System.out.println("*******Final Path Pose: "+ finalPose + " ********");
    addCommands(rollerCommand, command1, command2, command3, command4);
    //returnIntakeCommand);
}

//converts our inches into meters
private double inchesToMeters(double inch){
return inch/39.3701;
}

} // End of class