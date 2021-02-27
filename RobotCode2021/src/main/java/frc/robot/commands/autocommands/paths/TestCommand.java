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

import java.io.IOException;
import java.nio.file.Path;
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

public class TestCommand extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous.
     */
    private final double AUTO_SHOOTER_SPEED = 0.8;

    public TestCommand(SwerveDriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
            IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem towSubsystem,
            HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
            FlywheelPistonSubsystem fPistonSubsystem, TurretSubsystem turSubsystem) {
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

        String trajectoryJSON = Filesystem.getDeployDirectory() + "/Paths/GalaticSearch_A_Red.wpilib.json";
    Trajectory trajectory;
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //logger.log(Constants.LOG_LEVEL_INFO, "GalaticSearch_A_Red tragectory path: " + trajectoryPath.toString());
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        //logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSON);
        throw new RuntimeException("Unable to open trajectory: " + trajectoryJSON);
    }

        

        // -------- RAMSETE Commands -------- \\
        // Creates a command that can be added to the command scheduler in the sequential command
        // The Ramsete Controller is a trajectory tracker that is built in to WPILib.
        // This tracker can be used to accurately track trajectories with correction for minor disturbances.
        
        // This is our first atuo command this will run the drivetrain using the first trajectory we made

        SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory, dSubsystem::getPose, dSubsystem.getKinematics(), 
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

        // add commands here to run during auto
        addCommands(
        command1,
        new StopDriveCommand(dSubsystem)
        );
        //returnIntakeCommand);
    }

    //converts our inches into meters
    private double inchesToMeters(double inch){
        return inch/39.3701;
    }

} // End of Class
