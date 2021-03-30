package frc.robot.utilities;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import java.util.logging.*;
import frc.robot.Constants;

   
public class CameraUtil {

    
    //Logger
    private static final Logger logger = Logger.getLogger(CameraUtil.class.toString());

    public CameraUtil() {

    }
    // Starts the capture for the cameras
        public void startCapture() {
            logger.entering(CameraUtil.class.getName(), "setPosition()");
            logger.log(Constants.LOG_LEVEL_INFO, "Starting Cameras' Capture...");
            // creates a thread which runs concurrently with the program
            new Thread(() -> {

            // Instantiate the USB cameras and begin capturing their video streams
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
            //UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(Constants.UTIL_CAMERA_1_ID);
        
            // set the cameras' reolutions and FPS
            camera.setResolution(160, 120);
            camera.setFPS(30);
            //camera2.setResolution(Constants.CAMERA2_WIDTH, Constants.CAMERA2_HEIGHT);
            //camera2.setFPS(Constants.CAMERA2_FPS);

            }).start();
            logger.log(Constants.LOG_LEVEL_INFO, "Cameras' Capture Started!");
            logger.exiting(CameraUtil.class.getName(), "setPosition");

        }
}
