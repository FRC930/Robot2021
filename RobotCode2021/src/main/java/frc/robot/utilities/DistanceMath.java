package frc.robot.utilities;
import java.lang.Math;

import java.util.logging.*;
import frc.robot.Constants;


public class DistanceMath {
  public static final double GOAL_HEIGHT = 98;
  public static final double CAMERA_HEIGHT = 27.5455;
  public static final double CAMERA_ANGLE = 20;
  
  //Logger
  private static final Logger logger = Logger.getLogger(CameraUtil.class.toString());

  //Inputs: Goal Angle on Y Axis
  //Outputs: Distance from base of ground directly beneath camera to base of goal
  //Assumes Camerea Height to be equal to "CAMERA_HEIGHT" (Line 5)
  //Assumes Camera Angle to be equal to "CAMERA_ANGLE" (Line 6)
  //Ignores X Axis Angle. For most accurate results "tx" should be 0.
  public static double getDistY(double ty)
  {
    logger.entering(DistanceMath.class.getName(), "getDistY()");
    logger.log(Constants.LOG_LEVEL_FINER, "Starting Cameras' Capture...");
    logger.exiting(DistanceMath.class.getName(), "getDistY()");
    return (GOAL_HEIGHT-CAMERA_HEIGHT)/Math.tan(Math.toRadians(CAMERA_ANGLE+ty));
  }
  
  //Inputs: Goal Angle on Y Axis, Camera Angle
  //Outputs: Distance from base of ground directly beneath camera to base of goal
  //Assumes Camerea Height to be equal to "CAMERA_HEIGHT" (Line 5)
  //Ignores X Axis Angle. For most accurate results "tx" should be 0.
  public static double getDistY(double ty, double cameraAngle)
  {
    logger.entering(DistanceMath.class.getName(), "getDistY()");
    logger.log(Constants.LOG_LEVEL_FINER, "Starting Cameras' Capture...");
    logger.exiting(DistanceMath.class.getName(), "getDistY()");
    return (GOAL_HEIGHT-CAMERA_HEIGHT)/Math.tan(Math.toRadians(cameraAngle+ty));
  }
    
  //Inputs: Goal Angle on Y Axis, Camera Angle, Camera Height
  //Outputs: Distance from base of ground directly beneath camera to base of goal
  //Ignores X Axis Angle. For most accurate results "tx" should be 0.
  public static double getDistY(double ty, double cameraAngle, double cameraHeight)
  {
    logger.entering(DistanceMath.class.getName(), "getDistY()");
    logger.log(Constants.LOG_LEVEL_FINER, "Starting Cameras' Capture...");
    logger.exiting(DistanceMath.class.getName(), "getDistY()");
    return (GOAL_HEIGHT-cameraHeight)/Math.tan(Math.toRadians(cameraAngle+ty));
  }
}
