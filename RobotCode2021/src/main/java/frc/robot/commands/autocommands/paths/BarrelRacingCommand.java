/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.utilities.AutonConfig;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;

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

    // Generates a trajectory for a path to move towards furthest ball in trench run
    String trajectoryJSON = Filesystem.getDeployDirectory() + "/Paths/BarrelRacing.wpilib.json";
    Trajectory trajectory;
    try {
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         logger.log(Constants.LOG_LEVEL_INFO, "BarrelRacing tragectory path: " + trajectoryPath.toString());
         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     } catch (IOException ex) {
         DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
         logger.log(Constants.LOG_LEVEL_INFO, "Unable to open trajectory: " + trajectoryJSON);
         throw new RuntimeException("Unable to open trajectory: " + trajectoryJSON);
    }
// this is our config for how much power goes to the motors
var autoVoltageConstraint = AutonConfig.getInstance().getAutoVoltageConstraint();
//PID values for X/Y and rotation
double kPX = 1.1;
double kIX = 0;
double kDX = 0;
double kPY = /*1.1*/ 2;
double kIY = 0;
double kDY = 0;
double kPRot = 0;
double kIRot = 0;
double kDRot = 0;
double maxV = Math.PI * 2;
double maxA = Math.PI;

// -------- Trajectories -------- \\
// Generates a trajectory 

// -------- RAMSETE Commands -------- \\
// Creates a command that can be added to the command scheduler in the sequential command
// The Ramsete Controller is a trajectory tracker that is built in to WPILib.
// This tracker can be used to accurately track trajectories with correction for minor disturbances.

// This is our first atuo command this will run the drivetrain using the first trajectory we made

SwerveControllerCommand command1 = new SwerveControllerCommand(trajectory, dSubsystem::getPose, dSubsystem.getSwerveKinematics(), 
    new PIDController(kPX, kIX, kDX), new PIDController(kPY, kIY, kDY), new ProfiledPIDController(kPRot, kIRot, kDRot,
    new TrapezoidProfile.Constraints(maxV, maxA)), dSubsystem::swerveDrive, dSubsystem);


/*
Path Description:
-----------------
- Drive off intiation line
- Move to the side 2 Rendezvous Point balls
- Pick up two rendezvous point balls
- Shoot all 5 balls held
*/
Pose2d finalPose = trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters;
System.out.println("*******Initial Pose: "+ trajectory.getInitialPose() + " ********");
dSubsystem.resetPose(trajectory.getInitialPose());
System.out.println("*******Adjusted First Pose: " + dSubsystem.getPose() + "********");
System.out.println("*******Final Pose: "+ finalPose + " ********");
addCommands(command1);
//returnIntakeCommand);
}

//converts our inches into meters
private double inchesToMeters(double inch){
return inch/39.3701;
}

} // End of class
