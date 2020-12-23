package frc.robot.utilities;
import java.lang.Math;

public class DistanceMath {
  public static final double GOAL_HEIGHT = 98;
  public static final double CAMERA_HEIGHT = 24.9; 
  public static final double CAMERA_ANGLE = 20;

  //Inputs: Goal Angle on Y Axis
  //Outputs: Distance from base of ground directly beneath camera to base of goal
  //Assumes Camerea Height to be equal to "CAMERA_HEIGHT" (Line 5)
  //Assumes Camera Angle to be equal to "CAMERA_ANGLE" (Line 6)
  //Ignores X Axis Angle. For most accurate results "tx" should be 0.
  public static double getDistY(double ty)
  {
    return (GOAL_HEIGHT-CAMERA_HEIGHT)/Math.tan(Math.toRadians(CAMERA_ANGLE+ty));
  }
  
  //Inputs: Goal Angle on Y Axis, Camera Angle
  //Outputs: Distance from base of ground directly beneath camera to base of goal
  //Assumes Camerea Height to be equal to "CAMERA_HEIGHT" (Line 5)
  //Ignores X Axis Angle. For most accurate results "tx" should be 0.
  public static double getDistY(double ty, double cameraAngle)
  {
    return (GOAL_HEIGHT-CAMERA_HEIGHT)/Math.tan(Math.toRadians(cameraAngle+ty));
  }
    
  //Inputs: Goal Angle on Y Axis, Camera Angle, Camera Height
  //Outputs: Distance from base of ground directly beneath camera to base of goal
  //Ignores X Axis Angle. For most accurate results "tx" should be 0.
  public static double getDistY(double ty, double cameraAngle, double cameraHeight)
  {
    return (GOAL_HEIGHT-cameraHeight)/Math.tan(Math.toRadians(cameraAngle+ty));
  }
}
