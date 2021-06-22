package frc.robot.utilities;

import edu.wpi.first.wpilibj.Preferences;

/*
   https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/smartdashboard/smartdashboard-intro.html
   Smartdashboard setup
   - In Simulation mode
   -- Tools->Preferences->Team Number/Host = locathost
   -- (if values not on already smartdashboard) View->Add...->Robot Preferences
   --- TeamNumber(Number) = 930 or 9930  (WHEN SMARTDASHBOARD CONNECTED!)
   After that can shutdown dashboard!!  Values saved in simulator (i think)
   - In Robot mode
   -- Tools->Preferences->Team Number/Host = 930  or 9930 
   -- (if values not on already smartdashboard) View->Add...->Robot Preferences
   --- TeamNumber(Number) = 930 or 9930  (WHEN SMARTDASHBOARD CONNECTED!)
   After that can shutdown dashboard!!  Values saved in RoboIO (i think)
*/
public class RobotPreferences {
    private static RobotPreferences instance;
    private Preferences pref;
    int teamNumber;

    public RobotPreferences() {
        pref = Preferences.getInstance();
        teamNumber = pref.getInt("TeamNumber", 930);
        // TODO Add more preferences (get create a getter and private field)
    }

    public Preferences getPreferences() {
        return pref;
    }

    public int getTeamNumber() {
        return teamNumber;
    }

    // Singleton
    public static RobotPreferences getInstance() {
        if (instance == null) {
            instance = new RobotPreferences();
        }
        return instance;
    }
}