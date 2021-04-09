package frc.robot.utilities;

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

    /**
   * returning b
   * @param targetX  The X postion of the controller  (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    private  double calculateFrontAxle(double targetX, double rotation) {
        double Rtn = targetX + rotation * (wheelBase / r);
        return Rtn;
    }

    /**
   * returns a
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    private  double calculateRearAxle(double targetX, double rotation) {
        double Rtn = targetX - rotation * (wheelBase / r);
        return Rtn;
    }

    /**
   * returns c
   * 
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
   private double calculateLeftTrack(double targetY, double rotation) {
        double Rtn = targetY - rotation * (trackWidth / r);
        return Rtn;
    }

    /**
   * returns d
   * 
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public  double calculateRightTrack(double targetY, double rotation) {
        double Rtn = targetY + rotation * (trackWidth / r);
        return Rtn;
    }
    
    
    //constructer 
    // --initializeing the radius 
    public SwerveMath(){
        wheelBase = 22.5;
        trackWidth = 24.5;
        
        r = Math.sqrt((wheelBase * wheelBase) + (trackWidth * trackWidth));
    }

    /**
   * gets back right speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getBackRightSpeed(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);
       
        double BRSpeed = Math.sqrt((a * a) + (d * d));

        return BRSpeed;
    }
    
    /**
   * gets back left speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getBackLeftSpeed(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double c =  calculateLeftTrack(targetY, rotation);
       
        double BLSpeed = Math.sqrt((a * a) + (c * c));

        return BLSpeed;
    }

    /**
   * gets front right speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getFrontRightSpeed(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);

        double FRSpeed = Math.sqrt((b * b) + (d * d)); 
        
        return FRSpeed;
    }

    /**
   * gets front left speed
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getFrontLeftSpeed(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double c = calculateLeftTrack(targetY, rotation);
       
        double FLSpeed = Math.sqrt((b * b) + (c * c)); 
        return FLSpeed;
    }

    /**
   * gets back right angle
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getBackRightAngle(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);

        double BRAngle = (Math.atan2(a, d) / Math.PI) * 180;

        return BRAngle;
    }

    /**
   * gets back left angle
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getBackLeftAngle(double targetX, double targetY, double rotation) {
        double a = calculateRearAxle(targetX, rotation);
        double c = calculateLeftTrack(targetY, rotation);
       
        double BLAngle = (Math.atan2(a, c) / Math.PI) * 180;

        return BLAngle;
    }

    /**
   * get front right angle
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getFrontRightAngle(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double d = calculateRightTrack(targetY, rotation);
       
        double FRAngle = (Math.atan2(b, d) / Math.PI) * 180;
             
        return FRAngle;
    }

    /**
   * gets front left axle
   * 
   * @param targetX  The X postion of the controller  (Left stick)
   * @param targetY  The Y position of the controller (Left stick)
   * @param rotation The Y position of the controller (Right stick)
   */
    public double getFrontLeftAngle(double targetX, double targetY, double rotation) {
        double b = calculateFrontAxle(targetX, rotation);
        double c = calculateLeftTrack(targetY, rotation);
            
        double FLAngle = (Math.atan2(b, c) / Math.PI) * 180;

        return FLAngle;
    }
}
