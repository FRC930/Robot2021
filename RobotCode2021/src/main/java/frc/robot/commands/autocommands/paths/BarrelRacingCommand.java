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
import frc.robot.utilities.AutonConfig;
import frc.robot.commands.hoppercommands.SetAutonomousHopperCommand;
import frc.robot.commands.hoppercommands.SetHopperCommand;
import frc.robot.commands.turretcommads.AutoTurretTurnCommand;

import frc.robot.commands.drivecommands.StopDriveCommand;
import frc.robot.commands.turretcommads.AutoAimAutonomousCommand;
import frc.robot.commands.shootercommands.StopTowerKickerCommandGroup;
import frc.robot.commands.shootercommands.flywheelcommands.DefaultFlywheelCommand;
import frc.robot.commands.shootercommands.flywheelcommands.RunFlywheelAutoCommand;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
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

        // -------- Trajectories -------- \\

        // Generates all trajectories for the path
        Trajectory trajectory1;
        Trajectory trajectory2;
        Trajectory trajectory3;
        Trajectory trajectory4;
        Trajectory trajectory5;
        Trajectory trajectory6;
        Trajectory trajectory7;
        Trajectory trajectory8;
        Trajectory trajectory9;
        String trajectory1JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing1.wpilib.json";
        String trajectory2JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing2.wpilib.json";
        String trajectory3JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing3.wpilib.json";
        String trajectory4JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing4.wpilib.json";
        String trajectory5JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing5.wpilib.json";
        String trajectory6JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing6.wpilib.json";
        String trajectory7JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing7.wpilib.json";
        String trajectory8JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing8.wpilib.json";
        String trajectory9JSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing/BarrelRacing9.wpilib.json";
        // try {
        //     logger.log(Constants.LOG_LEVEL_INFO, "BarrelRacing tragectory path: " + Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON));
        //     trajectory1 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON));
        //     trajectory2 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON));
        //     trajectory3 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory3JSON));
        //     trajectory4 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory4JSON));
        //     trajectory5 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory5JSON));
        //     trajectory6 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory6JSON));
        //     trajectory7 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory7JSON));
        //     trajectory8 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory8JSON));
        //     trajectory9 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectory9JSON));
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory: " + trajectory1JSON, ex.getStackTrace());
        //     logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectory1JSON);
        //     throw new RuntimeException("Unable to open trajectory: " + trajectory1JSON);
        // }
        //Offsets calculated on 3/25 meeting
        double xOffset = inchesToMeters(35.25);
        double yOffset = inchesToMeters(6.5);
        Transform2d pathOffset = new Transform2d(new Translation2d(xOffset, yOffset), new Rotation2d(0));
        //PID values for X/Y and rotation
        double kPX = /*1.1*/ 0;
        double kIX = 0;
        double kDX = 0;
        double kPY = /*2*/ 0;
        double kIY = 0;
        double kDY = 0;
        double kPRot = 0;
        double kIRot = 0;
        double kDRot = 0;
        double maxV = Math.PI * 2;
        double maxA = Math.PI;

        // -------- Trajectories -------- \\
        // Generates a trajectory 
        //Manually generated trajectory
        trajectory1 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(1.207, -2.528, new Rotation2d(Math.toRadians(28))),
             List.of( 
                 new Translation2d(3.089 + xOffset, -2.131 + yOffset)
             ),
             new Pose2d(4.5464 + xOffset, -2.3116 - yOffset, new Rotation2d(Math.toRadians(-62))),
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory2 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(4.5464 + xOffset, -2.3116 - yOffset, new Rotation2d(Math.toRadians(-62))),
             List.of( 
                 new Translation2d(4.648 + xOffset, -3.5 - yOffset)
             ),
             //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
             new Pose2d(4.523 + inchesToMeters(26) - xOffset, -3.683 - inchesToMeters(6) - yOffset, new Rotation2d(Math.toRadians(-180))), //X: was 130y is -135
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory3 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(4.523 + inchesToMeters(26) - xOffset, -3.683 - inchesToMeters(6) - yOffset, new Rotation2d(Math.toRadians(-180))),
             List.of( 
                 new Translation2d(3.9 - xOffset, -2.921 + yOffset)
             ),
             //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
             new Pose2d(4.5 + xOffset, -2.312 + yOffset, new Rotation2d(Math.toRadians(0))), //X: was 130y is -135
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory4 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(4.5 + xOffset, -2.312 + yOffset, new Rotation2d(Math.toRadians(0))),
             List.of( 
                 new Translation2d(5.046 + xOffset, -2.347 + yOffset)
             ),
             //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
             new Pose2d(6.908 + inchesToMeters(8) + xOffset, -1.69 + inchesToMeters(39) + yOffset, new Rotation2d(Math.toRadians(65))), //X: was 130y is -135
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory5 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(6.908 + inchesToMeters(8) + xOffset, -1.69 + inchesToMeters(38) + yOffset, new Rotation2d(Math.toRadians(65))),
                 List.of( 
                     new Translation2d(6.629 + inchesToMeters(37) - xOffset, -0.877 + inchesToMeters(73) + yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(6.0 + inchesToMeters(30) - xOffset, -1.563 + inchesToMeters(38) + yOffset, new Rotation2d(Math.toRadians(-90))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory6 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(6.0 + inchesToMeters(30) - xOffset, -1.563 + inchesToMeters(38) + yOffset, new Rotation2d(Math.toRadians(-90))),
                 List.of( 
                     new Translation2d(6.108 + inchesToMeters(10) + xOffset, -2.807 + inchesToMeters(38) - yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(7.277 + inchesToMeters(-4) + xOffset, -3.607 + inchesToMeters(32) - yOffset, new Rotation2d(Math.toRadians(-20))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory7 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(7.277 + inchesToMeters(-4) + xOffset, -3.607 + inchesToMeters(32) - yOffset, new Rotation2d(Math.toRadians(-20))),
                 List.of( 
                     //new Translation2d(8.381 + inchesToMeters(24) + xOffset, -3.264 + inchesToMeters(38) + yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(8.381 + inchesToMeters(24) + xOffset, -3.264 + inchesToMeters(18) + yOffset, new Rotation2d(Math.toRadians(-20))), //X: was 130y is -135
                // new Pose2d(7.861 + inchesToMeters(18) + xOffset, -2.299 + inchesToMeters(40) + yOffset, new Rotation2d(Math.toRadians(150))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory8 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(7.861 - xOffset, -2.299 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))),
                 List.of( 
                     new Translation2d(6.02 + xOffset, -2.312 - yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(4.598 - xOffset, -1.924 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory9 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(4.598 - xOffset, -1.924 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))),
                 List.of( 
                     new Translation2d(2.925 + xOffset, -1.803 - yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(1.277 - xOffset, -1.872 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
        /* Previous attempts to align paths:
        * trajectory1 = trajectory1.relativeTo(dSubsystem.getPose());
        * trajectory1 = trajectory1.transformBy(pathOffset);
        * trajectory1 = AutonConfig.getInstance().transformBy(trajectory1, pathOffset);
        */
        // -------- RAMSETE Commands -------- \\
        // Creates a command that can be added to the command scheduler in the sequential command
        // The Ramsete Controller is a trajectory tracker that is built in to WPILib.
        // This tracker can be used to accurately track trajectories with correction for minor disturbances.
            
        // This is our first atuo command this will run the drivetrain using the first trajectory we made
        SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory1, 
            dSubsystem::getPose, dSubsystem.getSwerveKinematics(), new PIDController(kPX, kIX, kDX), 
            new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot, 
            new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        SwerveControllerCommand command2 = new SwerveControllerCommand(trajectory2, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
            new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
            new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        SwerveControllerCommand command3 = new SwerveControllerCommand(trajectory3, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
            new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
            new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        SwerveControllerCommand command4 = new SwerveControllerCommand(trajectory4, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
             new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
             new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        SwerveControllerCommand command5 = new SwerveControllerCommand(trajectory5, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
             new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
             new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        SwerveControllerCommand command6 = new SwerveControllerCommand(trajectory6, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
             new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
             new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        SwerveControllerCommand command7 = new SwerveControllerCommand(trajectory7, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
            new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
            new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        // SwerveControllerCommand command8 = new SwerveControllerCommand(trajectory8, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
        //     new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
        //     new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);
        // SwerveControllerCommand command9 = new SwerveControllerCommand(trajectory9, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
        //     new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
        //     new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);

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
        addCommands(command1, command2, command3, command4, command5, command6, command7/*, command8, command9*/);
        //returnIntakeCommand);
    }
    //converts our inches into meters
    private double inchesToMeters(double inch){
        return inch/39.3701;
    }
} // End of class
