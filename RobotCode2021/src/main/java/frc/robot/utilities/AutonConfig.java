package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.*;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * DeadbandMath
 */
public class AutonConfig {

    // All measurements are in meters

    private SwerveDriveKinematicsConstraint autoVoltageConstraint;

    private TrajectoryConfig trajectoryConfig;

    private TrajectoryConfig reverseConfig;

    private TrajectoryConfig slowConfigStart;

    private TrajectoryConfig slowConfigContinue;

    private TrajectoryConfig slowConfigEnd;

    private final int endVelocity_SlowConfig = 2;

    // Static flag for singleton
    private static AutonConfig lastInstance = null;

    // Constructs logger
    private static final Logger logger = Logger.getLogger(AutonConfig.class.toString());

    // ------------- CONSTANTS --------------- //
    public static final double PX = 0;
    public static final double IX = 0;
    public static final double DX = 0;
    public static final double PY = 0;
    public static final double IY = 0;
    public static final double DY = 0;
    public static final double PROT = 3;
    public static final double IROT = 0;
    public static final double DROT = 0;
    public static final double MAXV = Math.PI * 2;
    public static final double MAXA = Math.PI;

    public static double xOffset = inchesToMeters(35.25);
    public static double yOffset = inchesToMeters(6.5);

    // Class constructor - sets logger lever
    private AutonConfig(DriveSubsystem dSubsystem) {
        autoVoltageConstraint = new SwerveDriveKinematicsConstraint(dSubsystem.getSwerveKinematics(),
                Constants.KMAXSPEED);
        // Configurate the values of all trajectories for max velocity and acceleration
        trajectoryConfig = new TrajectoryConfig(5.5, 4.25)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(dSubsystem.getSwerveKinematics()).setEndVelocity(5.5)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        reverseConfig = new TrajectoryConfig(3, 1)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(dSubsystem.getSwerveKinematics()).setEndVelocity(0)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint).setReversed(true);

        // Used to be maxVel=3, maxAcc=4
        // Need to fix mechanically
        slowConfigStart = new TrajectoryConfig(3, 2)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(dSubsystem.getSwerveKinematics()).setEndVelocity(endVelocity_SlowConfig)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        slowConfigContinue = new TrajectoryConfig(3, 4)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(dSubsystem.getSwerveKinematics()).setEndVelocity(endVelocity_SlowConfig)
                .setStartVelocity(endVelocity_SlowConfig)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        slowConfigEnd = new TrajectoryConfig(3, 4)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(dSubsystem.getSwerveKinematics()).setStartVelocity(endVelocity_SlowConfig)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
    }

    public static void initInstance(DriveSubsystem dSubsystem) {
        lastInstance = new AutonConfig(dSubsystem);
    }

    // Method to get an instance of AutonConfig
    public static AutonConfig getInstance() {
        if (lastInstance == null) {
            logger.log(Level.SEVERE, "need to call init config");
            throw new IllegalArgumentException("need to call init config: AutonConfig.initInstance()");
        }
        return lastInstance;
    }

    public SwerveDriveKinematicsConstraint getAutoVoltageConstraint() {
        return autoVoltageConstraint;
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return trajectoryConfig;
    }

    public TrajectoryConfig getTrajectoryConfig(double startVelocity, double endVelocity) {
        return trajectoryConfig.setStartVelocity(startVelocity).setEndVelocity(endVelocity);
    }

    public TrajectoryConfig getReverseConfig() {
        return reverseConfig;
    }

    public TrajectoryConfig getSlowConfigStart() {
        return slowConfigStart;
    }

    public TrajectoryConfig getSlowConfigContinue() {
        return slowConfigContinue;
    }

    public TrajectoryConfig getSlowConfigEnd() {
        return slowConfigEnd;
    }

    // Custom transformBy from the trajectory class
    public Trajectory transformBy(Trajectory trajectory, Transform2d transform) {
        var firstState = trajectory.getStates().get(0);
        var firstPose = firstState.poseMeters;

        // Calculate the transformed first pose.
        List<State> newStates = new ArrayList<>();

        newStates.add(new State(firstState.timeSeconds, firstState.velocityMetersPerSecond,
                firstState.accelerationMetersPerSecondSq, firstPose, firstState.curvatureRadPerMeter));

        for (int i = 1; i < trajectory.getStates().size(); i++) {
            var state = trajectory.getStates().get(i);
            // We are transforming relative to the coordinate frame of the new initial pose.
            newStates.add(new State(state.timeSeconds, state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq, state.poseMeters.plus(transform), state.curvatureRadPerMeter));
        }

        return new Trajectory(newStates);
    }

    /**
     * <h3>inchesToMeters</h3>
     * 
     * @param inches a distance in inches
     * @return the same distance, converted to meters
     */
    public static double inchesToMeters(double inches) {
        return inches / 39.3701;
    }
}