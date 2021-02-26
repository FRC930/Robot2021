package frc.robot.utilities;
import frc.robot.Constants;

// We are using imperial (inches)
public class SwerveMath {
    // Distance between the front and back wheels
    private final double wheelBase;
 // Distance between left and right wheels
    private final double trackWidth; 
    //radius: length from middle of robot to the wheel
    private double r;

    /*
    *                FRONT
    * 
    *            c          d
    *            | 		    |
    *       b ------------------ b
    *            |          |
    *            |          |
    * LEFT       |          |      RIGHT
    *            |          |
    *            |          |
    *       a ------------------ a
    *            |          |
    *            c          d
    * 
    *                BACK
    */

    // returning b
    private  double calculateFrontAxle(double targetX, double rotation) {
        double Rtn = targetX + rotation * (wheelBase / r);
        return Rtn;
    }

    //returning a
    private  double calculateRearAxle(double targetX, double rotation) {
        double Rtn = targetX - rotation * (wheelBase / r);
        return Rtn;
    }

    //returning c
   private double calculateLeftTrack(double targetY, double rotation) {
        double Rtn = targetY - rotation * (trackWidth / r);
        return Rtn;
    }

    //returning d
    public  double calculateRightTrack(double targetY, double rotation) {
        double Rtn = targetY + rotation * (trackWidth / r);
        return Rtn;
    }
    
    
    //constructer 
    // --initializeing the radius 
    public SwerveMath(){
        wheelBase = 9.25 * 2.0; // 22.5; //  TODO 9.25 * 2.0 
        trackWidth = 10.625 * 2.0; // 24.5; // TODO 10.625 * 2.0
        
        r = Math.sqrt((wheelBase * wheelBase) + (trackWidth * trackWidth));
    }

    //getting back right speed
    public double getBackRightSpeed(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);
       
        double BRSpeed = Math.sqrt((a * a) + (d * d));

        return BRSpeed;
    }
    
    //getting back left speed
    public double getBackLeftSpeed(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double c =  calculateLeftTrack(targetY, rotation);
       
        double BLSpeed = Math.sqrt((a * a) + (c * c));

        return BLSpeed;
    }

    //getting front right speed
    public double getFrontRightSpeed(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);

        double FRSpeed = Math.sqrt((b * b) + (d * d)); 
        
        return FRSpeed;
    }

    //getting front left speed
    public double getFrontLeftSpeed(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double c = calculateLeftTrack(targetY, rotation);
       
        double FLSpeed = Math.sqrt((b * b) + (c * c)); 
        return FLSpeed;
    }

    //getting back right angle
    public double getBackRightAngle(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);

        double BRAngle = (Math.atan2(a, d) / Math.PI) * 180.0;

        return BRAngle;
    }

    //getting back left angle
    public double getBackLeftAngle(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double c = calculateLeftTrack(targetY, rotation);
       
        double BLAngle = (Math.atan2(a, c) / Math.PI) * 180.0;

        return BLAngle;
    }

    //getting front right angle
    public double getFrontRightAngle(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);
       
        double FRAngle = (Math.atan2(b, d) / Math.PI) * 180.0;
             
        return FRAngle;
    }

    //getting front left axle
    public double getFrontLeftAngle(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double c = calculateLeftTrack(targetY, rotation);
            
        double FLAngle = (Math.atan2(b, c) / Math.PI) * 180.0;

        return FLAngle;
    }
}
