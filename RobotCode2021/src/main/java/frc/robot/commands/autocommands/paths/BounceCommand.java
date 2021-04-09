/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
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
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;

import java.util.logging.*;

// -------- PATH DESCRIPTION -------- \\
// Alliance Side - Initial 3 & Trench 3 & Rendezvous 2

public class BounceCommand extends SequentialCommandGroup {

    private static final Logger logger = Logger.getLogger(BounceCommand.class.getName());
    private Trajectory trajectory1;
    private    Trajectory trajectory2;
    private    Trajectory trajectory3;
    private    Trajectory trajectory4;
    private    Trajectory trajectory5;
    private    Trajectory trajectory6;
    private    Trajectory trajectory7;
    private double xOffset = inchesToMeters(35.25);
    private double yOffset = inchesToMeters(6.5);
  /**
   * Path Description: ----------------- Shoot 3 from initiation line move through
   * trench to grab 3 balls Shoot 3 from trench position
   */
  public BounceCommand(SwerveDriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
      IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem tSubsystem,
      HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
      FlywheelPistonSubsystem fPistonSubsystem) {

    // -------- Trajectories -------- \\

    // Generates a trajectory for a path to move towards furthest ball in trench run
    trajectory1 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(1.243, -2.235, new Rotation2d(Math.toRadians(27.869698473156408))),
             List.of( 
                 new Translation2d(1.916 + xOffset, -2.123 + yOffset)
             ),
             new Pose2d(2.278 + xOffset, -0.898 + yOffset, new Rotation2d(Math.toRadians(27.869698473156408))),
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory2 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(2.278 + xOffset, -0.898 + yOffset, new Rotation2d(Math.toRadians(27.869698473156408))),
             List.of( 
                 new Translation2d(2.537 + xOffset, -2.209 - yOffset)
             ),
             //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
             new Pose2d(3.416 - xOffset, -3.632 - yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );

            trajectory3 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(3.416 - xOffset, -3.632 - yOffset, new Rotation2d(Math.toRadians(27.869698473156408))),
             List.of( 
                 new Translation2d(4.469 - xOffset, -2.373 + yOffset)
             ),
             //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
             new Pose2d(4.581 + xOffset, -0.863 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory4 = TrajectoryGenerator.generateTrajectory(
            // Robot starts at X: 0 Y: 0 and a rotation of 0 
             new Pose2d(4.581 + xOffset, -0.863 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))),
             List.of( 
                 new Translation2d(4.701 + xOffset, -2.416 + yOffset)
             ),
             //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
             new Pose2d(4.986 + xOffset, -3.468 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
             // Pass config
             AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory5 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(4.986 + xOffset, -3.468 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))),
                 List.of( 
                     new Translation2d(6.245 - xOffset, -3.649 - yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(6.814 - xOffset, -2.347 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory6 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(6.814 - xOffset, -2.347 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))),
                 List.of( 
                     new Translation2d(6.832 + xOffset, -0.872 - yOffset)
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(7.22 + xOffset, -1.821 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
            trajectory7 = TrajectoryGenerator.generateTrajectory(
                // Robot starts at X: 0 Y: 0 and a rotation of 0 
                new Pose2d(7.277 - xOffset, -3.607 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))),
                 List.of( 
                 ),
                 //this is our end point we end our first trajectory at X: 80 inches Y:-80 inches and -65 degrees from orgin
                 new Pose2d(8.35 + xOffset, -2.355 + yOffset, new Rotation2d(Math.toRadians(-97.978459709963636))), //X: was 130y is -135
                 // Pass config
                 AutonConfig.getInstance().getTrajectoryConfig()
            );
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

// -------- RAMSETE Commands -------- \\
// Creates a command that can be added to the command scheduler in the sequential command
// The Ramsete Controller is a trajectory tracker that is built in to WPILib.
// This tracker can be used to accurately track trajectories with correction for minor disturbances.

// This is our first atuo command this will run the drivetrain using the first trajectory we made

SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory1, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);

SwerveControllerCommand command2 = new SwerveControllerCommand(trajectory2, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);

SwerveControllerCommand command3 = new SwerveControllerCommand(trajectory3, dSubsystem::getPose, dSubsystem.getKinematics(), 
    new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kP, kI, kD,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::drive, dSubsystem);


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
    addCommands(command1, command2/*, command3, command4, command5, command6, command7*/);
    //returnIntakeCommand);
}

//converts our inches into meters
private double inchesToMeters(double inch){
return inch/39.3701;
}

} // End of class
