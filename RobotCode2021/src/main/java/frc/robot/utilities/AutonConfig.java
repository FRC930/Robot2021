package frc.robot.utilities;

import java.util.logging.*;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * DeadbandMath
 */
public class AutonConfig {
    
    //All measurements are in meters
    private final double BALL_RADIUS = 0.0889;

    private SwerveDriveKinematicsConstraint autoVoltageConstraint;

    private TrajectoryConfig trajectoryConfig;

    private TrajectoryConfig reverseConfig;

    private TrajectoryConfig slowConfig;

    //Static flag for singleton
    private static AutonConfig lastInstance = null;

    //Constructs logger
    private static final Logger logger = Logger.getLogger(AutonConfig.class.toString());
    
    //Class constructor - sets logger lever
    private AutonConfig(DriveSubsystem dSubsystem) {
        autoVoltageConstraint = new SwerveDriveKinematicsConstraint(dSubsystem.getSwerveKinematics(), Constants.KMAXSPEED);
        // Configurate the values of all trajectories for max velocity and acceleration
        trajectoryConfig =
        new TrajectoryConfig(2, 1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(dSubsystem.getSwerveKinematics())
        .setEndVelocity(1)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

        reverseConfig = new TrajectoryConfig(2, 1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(dSubsystem.getSwerveKinematics())
        .setEndVelocity(1)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(true);

        slowConfig = new TrajectoryConfig(2, 1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(dSubsystem.getSwerveKinematics())
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
    }

    public static void initInstance(DriveSubsystem dSubsystem) {
        lastInstance = new AutonConfig(dSubsystem); 
     }

	//Method to get an instance of AutonConfig
    public static AutonConfig getInstance() {
        if (lastInstance == null) {
            logger.log(Level.SEVERE, "need to call init config");
            throw new IllegalArgumentException("need to call init config: AutonConfig.initInstance()");
        }
        return lastInstance;
    }

    public SwerveDriveKinematicsConstraint getAutoVoltageConstraint(){
        return autoVoltageConstraint;
    }

    public TrajectoryConfig getTrajectoryConfig(){
        return trajectoryConfig;
    }

    public TrajectoryConfig getReverseConfig(){
        return reverseConfig;
    }

    public TrajectoryConfig getSlowConfig(){
        return slowConfig;
    }
}