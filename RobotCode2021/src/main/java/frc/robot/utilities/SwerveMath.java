package frc.robot.utilities;
import frc.robot.Constants;

// We are using imperial (inches)
public class SwerveMath {
    // Distance between the front and back wheels
    private final double wheelBase = 21.5;

    // Distance between left and right wheels
    private final double trackWidth = 19.125;

    private double r;

    public SwerveMath(){
        r = Math.sqrt((wheelBase * wheelBase) + (trackWidth * trackWidth));
    }

    public double getBackRightSpeed(double targetX, double targetY, double rotation) {
        double a = targetX - rotation * (wheelBase / r);
        double d = targetY + rotation * (trackWidth / r);

        double BRSpeed = Math.sqrt((a * a) + (d * d));

        return BRSpeed;
    }

    public double getBackLeftSpeed(double targetX, double targetY, double rotation) {
        double a = targetX - rotation * (wheelBase / r);
        double c = targetY - rotation * (trackWidth / r);

        double BLSpeed = Math.sqrt((a * a) + (c * c));

        return BLSpeed;
    }

    public double getFrontRightSpeed(double targetX, double targetY, double rotation) {
        double b = targetX + rotation * (wheelBase / r);
        double d = targetY + rotation * (trackWidth / r);

        double FRSpeed = Math.sqrt((b * b) + (d * d)); 
        
        return FRSpeed;
    }

    public double getFrontLeftSpeed(double targetX, double targetY, double rotation) {
        double b = targetX + rotation * (wheelBase / r);
        double c = targetY - rotation * (trackWidth / r);

        double FLSpeed = Math.sqrt((b * b) + (c * c)); 
        return FLSpeed;
    }

    public double getBackRightAngle(double targetX, double targetY, double rotation) {
        double a = targetX - rotation * (wheelBase / r);
        double d = targetY + rotation * (trackWidth / r);

        double BRAngle = (Math.atan2(a, d) / Math.PI) * 180;

        return BRAngle;
    }

    public double getBackLeftAngle(double targetX, double targetY, double rotation) {
        double a = targetX - rotation * (wheelBase / r);
        double c = targetY - rotation * (trackWidth / r);

        double BLAngle = (Math.atan2(a, c) / Math.PI) * 180;

        return BLAngle;
    }

    public double getFrontRightAngle(double targetX, double targetY, double rotation) {
        double b = targetX + rotation * (wheelBase / r);
        double d = targetY + rotation * (trackWidth / r);

        double FRAngle = (Math.atan2(b, d) / Math.PI) * 180;
        
        return FRAngle;
    }

    public double getFrontLeftAngle(double targetX, double targetY, double rotation) {
        double b = targetX + rotation * (wheelBase / r);
        double c = targetY - rotation * (trackWidth / r);

        double FLAngle = (Math.atan2(b, c) / Math.PI) * 180;

        return FLAngle;
    }
}
