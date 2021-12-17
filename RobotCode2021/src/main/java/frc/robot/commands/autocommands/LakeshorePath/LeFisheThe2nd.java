/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.LakeshorePath;

// Import as static to save keystrokes
import static frc.robot.utilities.AutonConfig.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.shootercommands.ShootPowerCellCommandGroup;
import frc.robot.commands.shootercommands.StopTowerKickerCommandGroup;
import frc.robot.commands.turretcommads.AutoAimAutonomousCommand;
import frc.robot.commands.turretcommads.AutoTurretTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.AutonConfig;

// -------- PATH DESCRIPTION -------- \\
// Alliance Side - Initial 3

/**
 * <h3>LeFisheThe2nd</h3>
 * 
 * This class loads the hopper full of power cells and shoots them from the
 * rendezvous point
 */
public class LeFisheThe2nd extends SequentialCommandGroup {
        private Trajectory trajectory1;

        // Path Description:
        // -----------------
        // Drives forward from initiation line
        // Shoots the balls already loaded.

        /**
         * <h3>LeFisheThe2nd</h3>
         * 
         * Autonomous command to run LeFisheThe2nd path
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
        public LeFisheThe2nd(DriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
                        IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem tSubsystem,
                        HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
                        FlywheelPistonSubsystem fPistonSubsystem, TurretSubsystem turSubsystem) {

                // -------- Trajectories -------- \\

                // Generates a trajectory for a path to move towards furthest ball in trench run
                trajectory1 = TrajectoryGenerator.generateTrajectory(
                                // Robot starts at X: 0 Y: 0 and a rotation of 0
                                new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), List.of(),
                                new Pose2d(inchesToMeters(24) + xOffset, 0 - yOffset,
                                                new Rotation2d(Math.toRadians(0))),
                                // Pass config
                                AutonConfig.getInstance().getSlowConfigEnd());

                // -------- RAMSETE Commands -------- \\
                // Creates a command that can be added to the command scheduler in the
                // sequential command
                // The Ramsete Controller is a trajectory tracker that is built in to WPILib.
                // This tracker can be used to accurately track trajectories with correction for
                // minor disturbances.

                // This is our first auto command this will run the drivetrain using the first
                // trajectory we made

                SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory1, dSubsystem::getPose,
                                dSubsystem.getSwerveKinematics(), new PIDController(PX, IX, DX),
                                new PIDController(PY, IY, DY),
                                new ProfiledPIDController(PROT, IROT, DROT,
                                                new TrapezoidProfile.Constraints(MAXV, MAXA)),
                                () -> Rotation2d.fromDegrees(0), dSubsystem::swerveDrive, dSubsystem);

                // Path Description:
                // -----------------
                // - Drive off initiation line
                // - Move to the side 2 Rendezvous Point balls
                // - Pick up two rendezvous point balls
                // - Shoot all 5 balls held
                Pose2d finalPose = trajectory1.getStates().get(trajectory1.getStates().size() - 1).poseMeters;
                System.out.println("*******First Robot Pose: " + dSubsystem.getPose() + "********");
                System.out.println("*******Initial Path Pose: " + trajectory1.getInitialPose() + " ********");
                System.out.println("*******Adjusted First Robot Pose: " + dSubsystem.getPose() + "********");
                System.out.println("*******Final Path Pose: " + finalPose + " ********");
                addCommands(command1, new WaitCommand(4), new AutoTurretTurnCommand(turSubsystem),
                                new ParallelRaceGroup(new WaitCommand(2), new AutoAimAutonomousCommand(lLightSubsystem,
                                                turSubsystem,
                                                new PIDController(Constants.TURRET_P, Constants.TURRET_I,
                                                                Constants.TURRET_D))),
                                new ParallelRaceGroup(new WaitCommand(2),
                                                new ShootPowerCellCommandGroup(tSubsystem, hSubsystem, kSubsystem)),
                                new StopTowerKickerCommandGroup(tSubsystem, kSubsystem));
        }
} // End of class