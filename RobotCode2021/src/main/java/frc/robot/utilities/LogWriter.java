package frc.robot.utilities;
import frc.robot.Constants;
import java.util.logging.Logger;

//Delete Soon!

class logWriter {
    //--------------------------------------------
    // Creates A Logger for Use. 
    // 
    // logWriter(String name) - Constructor
    //
    // void log(String level, String srcMethod, String log) -
    // Writes a new log reading the contents of log with the 
    // sourceMethod being srcMethod, and level of level.
    //--------------------------------------------



    //Examples of Use:

    // private static final Logger logger = Logger.getLogger(ClimberArmSubsystem.class.getName());
    //
    // becomes
    //
    // logWriter logger1 = new logWriter(ClimberArmSubsystem.class.getName());

    // logger.entering(ClimberArmSubsystem.class.getName(), "getSpeed");
    // logger.log(Constants.LOG_LEVEL_INFO, "Actual Motor Speed: " + climberArmMotor.getMotorOutputPercent());
    // logger.exiting(ClimberArmSubsystem.class.getName(), "getSpeed");
    //
    // becomes
    // 
    // logger1.log("INFO", "getSpeed", "Actual Motor Speed: " + climberArmMotor.getMotorOutputPercent());
    //



    //innitialization
    private String name;
    private Logger logger;

    //Constructor
    public logWriter(String name) {
        this.name = name;
        logger = Logger.getLogger(name);
    }

    public void log(String level, String srcMethod, String log) {
        logger.entering(name, srcMethod);
        switch (level) {
            case "INFO":
            logger.log(Constants.LOG_LEVEL_INFO, log);
              break;
            case "WARNING":
            logger.log(Constants.LOG_LEVEL_WARNING, log);
              break;
            case "FINE":
            logger.log(Constants.LOG_LEVEL_FINE, log);
              break;
            case "FINER":
            logger.log(Constants.LOG_LEVEL_FINER, log);
              break;
            case "FINEST":
            logger.log(Constants.LOG_LEVEL_FINEST, log);
              break;
          }
        logger.exiting(name, srcMethod);
    }
}