package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelPistonSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utilities.DeadbandMath;
import frc.robot.utilities.DistanceMath;
import frc.robot.utilities.DeadbandMath.DeadbandZone;
import frc.robot.utilities.DeadbandMath.ShotChance;
import frc.robot.utilities.ShooterMath;
import frc.robot.utilities.ShooterMath.ShotOutcome;;

/**
 * CheckIfShotPossible
 */
public class CheckIfShotPossible extends CommandBase{
    private DeadbandMath deadbandMath = DeadbandMath.getInstance();
    private ShooterMath shooterMath = ShooterMath.getInstance();
    private LimelightSubsystem limeLightSubsystem;
    private FlywheelPistonSubsystem flywheelPistonSubsystem;
    private ShotOutcome shotOutcome;
    private DeadbandZone deadbandZone;
    private ShotChance shotChance;
    private double angle;

    public CheckIfShotPossible(LimelightSubsystem lLightSubsystem, FlywheelPistonSubsystem fPistonSubsystem) {
        limeLightSubsystem = lLightSubsystem;
        flywheelPistonSubsystem = fPistonSubsystem;
        shotOutcome = ShotOutcome.NONE;
        deadbandZone = DeadbandZone.RED;
        shotChance = ShotChance.MISS;
    }
    
    
        

    @Override
    public boolean isFinished() {
        // TODO: Needs to be refractored because of top piston being gone
        /*
        if (flywheelPistonSubsystem.getTop()  && flywheelPistonSubsystem.getBottom()) {
            angle = 10.65;
        } 
        else if (flywheelPistonSubsystem.getTop() || flywheelPistonSubsystem.getBottom() ) {
            angle = 19.35;
        }
        else {
            angle = 30;
        }
        */

        //Set the shot type to the shooter.
        shooterMath.setPosition(angle, DistanceMath.getDistY(limeLightSubsystem.getVerticleOffset()));
        shotOutcome = shooterMath.getPossibleShot();
        
        deadbandMath.setPosition(limeLightSubsystem.getHorizontalOffset(), DistanceMath.getDistY(limeLightSubsystem.getVerticleOffset()));
        deadbandZone = deadbandMath.getDeadbandZone();
        shotChance = deadbandMath.getShotChance();

        // Return true if the shot types is best. We are not shooting at maybe or lower.
        if (this.shotOutcome == ShotOutcome.INNER && this.deadbandZone == DeadbandZone.GREEN && this.shotChance == ShotChance.HIGH) {
            return true;
        } 
        else {
            return false;
        }
    }
}