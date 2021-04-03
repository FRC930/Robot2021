package frc.robot.utilities;
import java.lang.Math;

import java.util.logging.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autocommands.paths.GalacticSearch_A_BlueCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipelines;

public class GalacticPathUtil {

    // An enum containing each path
    public static enum GalacticPath {
        RED_PATH_A,
        BLUE_PATH_A,
        RED_PATH_B,
        BLUE_PATH_B,
        NULL
    }
    private static GalacticPath galacPath = GalacticPath.NULL;

    //SequentialCommand[] PathAAuto = {GalacticSearch_A_RedCommand, GalacticSearch_A_BlueCommand}

    /**
    * Sets the current galactic path to follow
    * @param path Galactic path enum
    */
    public static void setGalacticPath(GalacticPath path) {
        galacPath = path;
    }

    /**
     * Returns the current galactic path enum
     * @param lSubsystem Current limelight object
     * @return
     */
    public static GalacticPath getAutonomousPath(LimelightSubsystem lSubsystem) {
        lSubsystem.setPipeline(LimelightPipelines.GALACTIC_PATH);

        // Offset of the ball from the crosshair on the limelight
        double hOffset = lSubsystem.getHorizontalOffset();
        double vOffset = lSubsystem.getVerticleOffset();

        // Path enum we want to return
        GalacticPath path;

        if((hOffset > 4.50 && hOffset < 8.50) && (vOffset > 10.50 && vOffset < 14.50)) {
            path = GalacticPathUtil.GalacticPath.RED_PATH_A;

        } else if((hOffset > 15.50 && hOffset < 18.50) && (vOffset > 10.50 && vOffset < 14.50)) {
            path = GalacticPathUtil.GalacticPath.BLUE_PATH_A;

        } else if((hOffset > 0.00 && hOffset < 2.50) && (vOffset > 4.50 && vOffset < 8.50)) {
            path = GalacticPathUtil.GalacticPath.RED_PATH_B;

        } else if((hOffset > 16.50 && hOffset < 20.50) && (vOffset > 11.50 && vOffset < 15.50)) {
            path = GalacticPathUtil.GalacticPath.BLUE_PATH_B;
        
        } else {
            path = GalacticPathUtil.GalacticPath.NULL;
        }

        return path;
    }
}
