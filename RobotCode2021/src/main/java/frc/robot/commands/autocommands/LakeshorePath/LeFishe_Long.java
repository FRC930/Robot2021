/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.LakeshorePath;

// Imported as static to save keystrokes
import static frc.robot.utilities.AutonConfig.*;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import frc.robot.commands.intakecommands.intakepistoncommands.ExtendIntakePistonCommand;
import frc.robot.commands.shootercommands.ShootPowerCellCommandGroup;
import frc.robot.commands.shootercommands.StopTowerKickerCommandGroup;
import frc.robot.commands.turretcommads.AutoAimAutonomousCommand;
import frc.robot.commands.turretcommads.AutoTurretTurnCommand;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.NewDriveSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.AutonConfig;

// -------- PATH DESCRIPTION -------- \\
// Alliance Side - Initial 3 & Trench 3 & Rendezvous 2

/**
 * <h3>LeFishe_Long</h3>
 * 
 * This class is an autonomous path for shooting and grabbing power cells
 * 
 */
public class LeFishe_Long extends SequentialCommandGroup {
        private Trajectory trajectory1;
        private Trajectory trajectory2;
        private Trajectory trajectory3;
        private Trajectory trajectory4;

        // Path Description:
        // -----------------
        // Shoot 3 from initiation line
        // Move through trench to grab 3 balls
        // Shoot 3 from trench position

        /**
         * <h3>LeFishe_Long</h3>
         * 
         * Autonomous command to run LeFishe_Long path
         * 
         * @param dSubsystem       the drive subsystem
         * @param iPistonSubsystem intake piston subsystem
         * @param iMotorSubsystem  intake motor subsystem
         * @param fSubsystem       flywheel subsystem
         * @param tSubsystem       tower subsystem
         * @param hSubsystem       hopper subsystem
         * @param kSubsystem       kicker subsystem
         * @param lLightSubsystem  limelight subsystem
         * @param fPistonSubsystem flywheel piston subsystem
         * @param turSubsystem     turret subsystem
         */
        public LeFishe_Long(NewDriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
                        IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem tSubsystem,
                        HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
                        FlywheelPistonSubsystem fPistonSubsystem, TurretSubsystem turSubsystem) {

                // -------- Trajectories -------- \\

                // NOT FOLLOWING JSON

                // HEADING IN TRAJECTORY CHANGES ANGLE THAT DRAWS THE LINE THAT THE ROBOT
                // FOLLOWS

                // REVERSE KINDA FUNKY THINK MORE

                // Generates a trajectory for a path to move towards furthest ball in trench run
                trajectory1 = TrajectoryGenerator.generateTrajectory(
                                // Robot starts at X: 0 Y: 0 and a rotation of 0
                                new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), List.of(),
                                new Pose2d(inchesToMeters(180) + xOffset, 0 + yOffset,
                                                new Rotation2d(Math.toRadians(0))),
                                // Pass config
                                AutonConfig.getInstance().getSlowConfigStart());

                trajectory2 = TrajectoryGenerator.generateTrajectory(
                                // Robot starts at X: 0 Y: 0 and a rotation of 0
                                new Pose2d(inchesToMeters(0), 0, new Rotation2d(Math.toRadians(0))), List.of(),
                                new Pose2d(inchesToMeters(-105) + xOffset, inchesToMeters(-80) + yOffset,
                                                new Rotation2d(Math.toRadians(0))),
                                // Pass config
                                AutonConfig.getInstance().getReverseConfig());

                trajectory3 = TrajectoryGenerator.generateTrajectory(
                                // Robot starts at X: 0 Y: 0 and a rotation of 0
                                new Pose2d(inchesToMeters(0), 0, new Rotation2d(Math.toRadians(-45))), List.of(),
                                new Pose2d(inchesToMeters(-5) + xOffset, inchesToMeters(-110) + yOffset,
                                                new Rotation2d(Math.toRadians(-45))),
                                // Pass config
                                AutonConfig.getInstance().getSlowConfigStart());

                trajectory4 = TrajectoryGenerator.generateTrajectory(
                                // Robot starts at X: 0 Y: 0 and a rotation of 0
                                new Pose2d(inchesToMeters(0), 0, new Rotation2d(Math.toRadians(135))), List.of(),
                                new Pose2d(inchesToMeters(-25) - xOffset, inchesToMeters(95) + yOffset,
                                                new Rotation2d(Math.toRadians(135))),
                                // Pass config
                                AutonConfig.getInstance().getReverseConfig()); // Configure the values of all
                                                                               // trajectories for max velocity and
                                                                               // acceleration

                // -------- RAMSETE Commands -------- \\
                // Creates a command that can be added to the command scheduler in the
                // sequential command
                // The Ramsete Controller is a trajectory tracker that is built in to WPILib.
                // This tracker can be used to accurately track trajectories with correction for
                // minor disturbances.

                // This is our first auto command this will run the drivetrain using the first
                // trajectory we made

                // Rotation2d.fromDegrees changes the actual rotation of the robot!!!!!
                SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory1, dSubsystem::getPose,
                                dSubsystem.getSwerveKinematics(), new PIDController(PX, IX, DX),
                                new PIDController(PY, IY, DY),
                                new ProfiledPIDController(PROT, IROT, DROT,
                                                new TrapezoidProfile.Constraints(MAXV, MAXA)),
                                () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

                SwerveControllerCommand command2 = new SwerveControllerCommand(trajectory2, dSubsystem::getPose,
                                dSubsystem.getSwerveKinematics(), new PIDController(PX, IX, DX),
                                new PIDController(PY, IY, DY),
                                new ProfiledPIDController(PROT, IROT, DROT,
                                                new TrapezoidProfile.Constraints(MAXV, MAXA)),
                                () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

                SwerveControllerCommand command3 = new SwerveControllerCommand(trajectory3, dSubsystem::getPose,
                                dSubsystem.getSwerveKinematics(), new PIDController(PX, IX, DX),
                                new PIDController(PY, IY, DY),
                                new ProfiledPIDController(PROT, IROT, DROT,
                                                new TrapezoidProfile.Constraints(MAXV, MAXA)),
                                () -> Rotation2d.fromDegrees(-60), dSubsystem::swerveDrive, dSubsystem);

                SwerveControllerCommand command4 = new SwerveControllerCommand(trajectory4, dSubsystem::getPose,
                                dSubsystem.getSwerveKinematics(), new PIDController(PX, IX, DX),
                                new PIDController(PY, IY, DY),
                                new ProfiledPIDController(PROT, IROT, DROT,
                                                new TrapezoidProfile.Constraints(MAXV, MAXA)),
                                () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

                RunIntakeMotorsCommand rollerCommand = new RunIntakeMotorsCommand(iMotorSubsystem);

                // Description:
                // -------------------------------------
                // Drive off initiation line
                // Move to the side 2 Rendezvous Point balls
                // Pick up two rendezvous point balls
                // Shoot all 5 balls held
                Pose2d finalPose = trajectory1.getStates().get(trajectory1.getStates().size() - 1).poseMeters;
                System.out.println("*******First Robot Pose: " + dSubsystem.getPose() + "********");
                System.out.println("*******Initial Path Pose: " + trajectory1.getInitialPose() + " ********");
                System.out.println("*******Adjusted First Robot Pose: " + dSubsystem.getPose() + "********");
                System.out.println("*******Final Path Pose: " + finalPose + " ********");
                // ADD PISTON RETRACT COMMAND AT END AND FLYWHEEL IS AT 50%! :)
                // TWEAK POINTS AND SPEED
                addCommands(rollerCommand, new ExtendIntakePistonCommand(iPistonSubsystem), command1, command2,
                                new AutoTurretTurnCommand(turSubsystem),
                                new ParallelRaceGroup(new WaitCommand(2), new AutoAimAutonomousCommand(lLightSubsystem,
                                                turSubsystem,
                                                new PIDController(Constants.TURRET_P, Constants.TURRET_I,
                                                                Constants.TURRET_D))),
                                new ParallelRaceGroup(new WaitCommand(2),
                                                new ShootPowerCellCommandGroup(tSubsystem, hSubsystem, kSubsystem)),
                                new StopTowerKickerCommandGroup(tSubsystem, kSubsystem), command3, command4,
                                new AutoTurretTurnCommand(turSubsystem),
                                new ParallelRaceGroup(new WaitCommand(2), new AutoAimAutonomousCommand(lLightSubsystem,
                                                turSubsystem,
                                                new PIDController(Constants.TURRET_P, Constants.TURRET_I,
                                                                Constants.TURRET_D))),
                                new ParallelRaceGroup(new WaitCommand(2),
                                                new ShootPowerCellCommandGroup(tSubsystem, hSubsystem, kSubsystem)),
                                new StopTowerKickerCommandGroup(tSubsystem, kSubsystem));
        }

} // End of class