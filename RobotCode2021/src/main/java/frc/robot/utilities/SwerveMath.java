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
    //
    //  positions of wheels a/b/c/d
    private double a, b, c, d;
    //
    //  flag indicating apply gyro
    private boolean useGyro = false;

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
    private  void calculateFrontAxle(double targetX, double rotation) {
        b = targetX + rotation * (wheelBase / r);
    }

    //returning a
    private  void calculateRearAxle(double targetX, double rotation) {
        a = targetX - rotation * (wheelBase / r);
    }

    //returning c
   private void calculateLeftTrack(double targetY, double rotation) {
        c = targetY - rotation * (trackWidth / r);
    }

    //returning d
    public  void calculateRightTrack(double targetY, double rotation) {
        d = targetY + rotation * (trackWidth / r);
    }
    
    
    //constructer 
    // --initializeing the radius 
    public SwerveMath(boolean IsUsingGyro){
        wheelBase = 9.25*2.0;
        trackWidth = 10.625*2.0;
        
        r = Math.sqrt((wheelBase * wheelBase) + (trackWidth * trackWidth));

        this.useGyro = IsUsingGyro;
    }

    //
    //  update a/b/c/d
    //  -- call this every loop of command to update wheel positions
    public void updateWheelPositions(double targetX, double targetY, double rotation, double gyroAngle)
    {
        double currentStr = targetX;
        double currentFwd = targetY;
        //
        //  update fwrd and straif if gyro is used
        if(useGyro)
        {
            double temp = currentFwd*Math.cos(gyroAngle) + currentStr*Math.sin(gyroAngle);
            currentStr = -currentFwd*Math.sin(gyroAngle) + currentStr*Math.cos(gyroAngle);
            currentFwd = temp;
        }

        //
        //  call functions
        calculateFrontAxle(currentStr, rotation);
        calculateLeftTrack(currentFwd, rotation);
        calculateRearAxle(currentStr, rotation);
        calculateRightTrack(currentFwd, rotation);
    }

    //getting back right speed
    public double getBackRightSpeed() { 
        double BRSpeed = Math.sqrt((a * a) + (d * d)); 
        //double BRSpeed = Math.sqrt((a * a) + (c * c));
        return BRSpeed;
    }
    
    //getting back left speed
    public double getBackLeftSpeed() {
        double BLSpeed = Math.sqrt((a * a) + (c * c));
        //double BLSpeed = Math.sqrt((a * a) + (d * d));
        return BLSpeed;
    }

    //getting front right speed
    public double getFrontRightSpeed() {
        double FRSpeed = Math.sqrt((b * b) + (d * d)); 
        //double FRSpeed = Math.sqrt((b * b) + (c * c)); 
        return FRSpeed;
    }

    //getting front left speed
    public double getFrontLeftSpeed() {
        double FLSpeed = Math.sqrt((b * b) + (c * c)); 
        //double FLSpeed = Math.sqrt((b * b) + (d * d)); 
        return FLSpeed;
    }

    //getting back right angle
    public double getBackRightAngle() {
        double BRAngle = (Math.atan2(a, d)  * 180 / Math.PI);
        //double BRAngle = (Math.atan2(a, c)  * 180 / Math.PI);
        return BRAngle;
    }

    //getting back left angle
    public double getBackLeftAngle() {
        double BLAngle = (Math.atan2(a, c) * 180 / Math.PI);
        //double BLAngle = (Math.atan2(a, d) * 180 / Math.PI);
        return BLAngle;
    }

    //getting front right angle
    public double getFrontRightAngle() {
        double FRAngle = (Math.atan2(b, d) * 180/ Math.PI) ;   
        //double FRAngle = (Math.atan2(b, c) * 180/ Math.PI) ;      
        return FRAngle;
    }

    //getting front left axle
    public double getFrontLeftAngle() {  
        double FLAngle = (Math.atan2(b, c) * 180 / Math.PI) ;
        //double FLAngle = (Math.atan2(b, d) * 180 / Math.PI) ;
        return FLAngle;
    }
}
