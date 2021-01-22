package frc.robot.utilities;
import frc.robot.Constants;
import java.util.logging.Logger;

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