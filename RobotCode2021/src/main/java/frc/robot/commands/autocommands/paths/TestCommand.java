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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelPistonSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import frc.robot.subsystems.TurretSubsystem;

public class TestCommand extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous.
     */

    public TestCommand(DriveSubsystem dSubsystem, IntakePistonSubsystem iPistonSubsystem,
            IntakeMotorSubsystem iMotorSubsystem, FlywheelSubsystem fSubsystem, TowerSubsystem towSubsystem,
            HopperSubsystem hSubsystem, KickerSubsystem kSubsystem, LimelightSubsystem lLightSubsystem,
            FlywheelPistonSubsystem fPistonSubsystem, TurretSubsystem turSubsystem) {
        // this is our config for how much power goes to the motors
        //PID values
        double kP = 0.2;
        double kI = 0;
        double kD = 0;
        double kPRot = 0.2;
        double kIRot = 0;
        double kDRot = 0;
        double maxV = Math.PI * 2;
        double maxA = Math.PI;
        // Configurate the values of all trajectories for max velocity and acceleration

        

        // -------- Trajectories -------- \\
        // Generates a trajectory 

    String trajectoryJSON = Filesystem.getDeployDirectory() + "/Paths/Test.wpilib.json";
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

        SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
            new PIDController(kP, kI, kD), new PIDController(kP, kI, kD), new ProfiledPIDController(kPRot, kIRot, kDRot,
            new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);

        

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
        command1
        );
        //returnIntakeCommand);
    }

} // End of Class
