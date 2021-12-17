/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.utilities;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;

//-------- CLASS --------\\
/**
 * <h4>ShuffleboardUtility</h4> This class manages all data displayed on the
 * Driver Station via Shuffleboard.
 */
public class ShuffleboardUtility {

    // -------- DECLARATIONS --------\\

    // Chooser objects for dropdowns
    private SendableChooser<Command> autonChooser;
    private SendableChooser<Command> shuffleboardChooser;

    // Intake data
    private boolean intakeIndicator;

    // Flywheel data
    private boolean shootIndicator;
    private boolean shooterUpToSpeed;
    private double flywheelSpeed;
    private double flywheelVoltage;
    private double velError;
    private double controlTol;
    private double modelAcc;
    private double encodAcc;
    private double maxVoltage;
    private double dtSeconds;
    private double shooterRPM;
    private boolean shooterAngle;

    // Miscellaneous data
    private boolean manualMode;
    private String shotType;
    private double gyroYaw;

    // Limelight data
    private double distanceFromTarget;

    // Turret data
    private double turretSpeed;
    private double turretEncoderPosition;

    // Hopper data
    private double hopperSpeed;

    // Endgame data
    private double endgameEncoderPosition;

    // Two tab setups
    private ShuffleboardTab testDebugTab;
    private ShuffleboardTab driverStationTab;

    // Entry nodes
    private NetworkTableEntry intakingEntry;
    private NetworkTableEntry shootingEntry;
    private NetworkTableEntry manualModeEntry;
    private NetworkTableEntry distanceFromTargetEntry;
    private NetworkTableEntry shotTypeEntry;
    private NetworkTableEntry shooterUpToSpeedEntry;
    private NetworkTableEntry turretSpeedEntry;
    private NetworkTableEntry hopperSpeedEntry;
    private NetworkTableEntry shooterAngleEntry;
    private NetworkTableEntry shooterRPMEntry;
    private NetworkTableEntry turretEncoderPositionEntry;
    private NetworkTableEntry gyroYawEntry;
    private NetworkTableEntry flywheelSpeedEntry;
    private NetworkTableEntry flywheelVoltageEntry;
    private NetworkTableEntry velErrorEntry;
    private NetworkTableEntry controlTolEntry;
    private NetworkTableEntry modelAccEntry;
    private NetworkTableEntry encodAccEntry;
    private NetworkTableEntry maxVoltageEntry;
    private NetworkTableEntry dtSecondsEntry;
    private NetworkTableEntry endgameEncoderPositionEntry;

    // Access flags for each group
    private boolean intakeAccess;
    private boolean hopperAccess;
    private boolean limelightAccess;
    private boolean flywheelAccess;
    private boolean turretAccess;
    private boolean endgameAccess;
    private boolean miscellaneousAccess;

    // Limelight object
    private HttpCamera limelightCamera;

    // Flag variable for singleton
    private static ShuffleboardUtility instance = null;

    // -------- CONSTRUCTOR --------\\

    /**
     * The constructor is made private to make a singleton. Only one instance is
     * allowed to be made.
     */
    private ShuffleboardUtility() {
        // Initialize all variables
        intakeIndicator = false;
        shootIndicator = false;
        manualMode = false;
        distanceFromTarget = 0.0;
        shotType = "";
        shooterUpToSpeed = false;
        flywheelSpeed = 0.0;
        flywheelVoltage = 0.0;
        velError = 0.0;
        controlTol = 0.0;
        modelAcc = 0.0;
        encodAcc = 0.0;
        maxVoltage = 12.0;
        dtSeconds = 0.0;
        hopperSpeed = 0.0;
        shooterRPM = 0.0;
        shooterAngle = false;
        turretSpeed = 0.0;
        turretEncoderPosition = 0.0;
        gyroYaw = 0.0;
        endgameEncoderPosition = 0.0;
        driverStationTab = Shuffleboard.getTab("Driver Station");
        testDebugTab = Shuffleboard.getTab("Testing & Debugging");
        intakingEntry = driverStationTab.add("Intaking?", intakeIndicator).getEntry();
        shootingEntry = driverStationTab.add("Shooting?", shootIndicator).getEntry();
        manualModeEntry = driverStationTab.add("Manual Mode?", manualMode).getEntry();
        distanceFromTargetEntry = driverStationTab.add("Distance from Target", distanceFromTarget).getEntry();
        shotTypeEntry = driverStationTab.add("Shot Type", shotType).getEntry();
        shooterUpToSpeedEntry = driverStationTab.add("Is Shooter up to speed?", shooterUpToSpeed).getEntry();
        turretSpeedEntry = testDebugTab.add("Turret Speed", turretSpeed).getEntry();
        hopperSpeedEntry = testDebugTab.add("Hopper Speed", hopperSpeed).getEntry();
        shooterAngleEntry = testDebugTab.add("Shooter Position (True = Far)", shooterAngle).getEntry();
        shooterRPMEntry = testDebugTab.add("Shooter RPM", shooterRPM).getEntry();
        turretEncoderPositionEntry = testDebugTab.add("Turret Encoder Position", turretEncoderPosition).getEntry();
        gyroYawEntry = testDebugTab.add("Gyro Yaw", gyroYaw).getEntry();
        flywheelSpeedEntry = driverStationTab.add("Flywheel Speed", flywheelSpeed).getEntry();
        flywheelVoltageEntry = driverStationTab.add("Flywheel Voltage", flywheelVoltage).getEntry();
        velErrorEntry = driverStationTab.add("Velocity Error", velError).getEntry();
        controlTolEntry = driverStationTab.add("Control Tolerance", controlTol).getEntry();
        maxVoltageEntry = driverStationTab.add("Max Voltage", maxVoltage).getEntry();
        modelAccEntry = driverStationTab.add("Model accuracy", modelAcc).getEntry();
        encodAccEntry = driverStationTab.add("Encoder Accuracy", encodAcc).getEntry();
        dtSecondsEntry = driverStationTab.add("DT Seconds", dtSeconds).getEntry();
        endgameEncoderPositionEntry = driverStationTab.add("Endgame Encoder", endgameEncoderPosition).getEntry();
        autonChooser = new SendableChooser<Command>();
        shuffleboardChooser = new SendableChooser<Command>();

        driverStationTab.add("Auton Path Selector", autonChooser);
        testDebugTab.add("Shuffleboard Detail Selector", shuffleboardChooser);

        intakeAccess = false;
        hopperAccess = false;
        limelightAccess = false;
        flywheelAccess = false;
        turretAccess = false;
        endgameAccess = false;
        miscellaneousAccess = false;

    }

    /**
     * getInstance only allows the creation of one instance of the class to be made.
     * 
     * @return singular instance of the ShuffleboardUtility class
     */
    public static ShuffleboardUtility getInstance() {
        // Checks for existing object
        if (instance == null) {
            instance = new ShuffleboardUtility();
        }
        return instance;
    }

    // ------ Data Access ------\\

    /**
     * Set Shuffleboard accessibility for each data group. "true" allows the value
     * to be displayed on the Shuffleboard.
     * 
     * @param _intakeAccess        set access for intake data
     * @param _hopperAccess        set access for hopper data
     * @param _limelightAccess     set access for limelight data
     * @param _flywheelAccess      set access for flywheel data
     * @param _turretAccess        set access for turret data
     * @param _endgameAccess       set access for endgame data
     * @param _miscellaneousAccess set access for miscellaneous data
     */
    public void setDataAccess(boolean _intakeAccess, boolean _hopperAccess, boolean _limelightAccess,
            boolean _flywheelAccess, boolean _turretAccess, boolean _endgameAccess, boolean _miscellaneousAccess) {
        intakeAccess = _intakeAccess;
        hopperAccess = _hopperAccess;
        limelightAccess = _limelightAccess;
        flywheelAccess = _flywheelAccess;
        turretAccess = _turretAccess;
        endgameAccess = _endgameAccess;
        miscellaneousAccess = _miscellaneousAccess;
    }

    /**
     * Sets accessibility for all groups to false.
     */
    public void allAccessFalse() {
        intakeAccess = false;
        hopperAccess = false;
        limelightAccess = false;
        flywheelAccess = false;
        turretAccess = false;
        endgameAccess = false;
        miscellaneousAccess = false;
    }

    /**
     * Toggles the accessibility for the intake group.
     */
    public void toggleIntakeAccess() {
        intakeAccess = !intakeAccess;
    }

    /**
     * Toggles the accessibility for the hopper group.
     */
    public void toggleHopperAccess() {
        hopperAccess = !hopperAccess;
    }

    /**
     * Toggles the accessibility for the limelight group.
     */
    public void toggleLimelightAccess() {
        limelightAccess = !limelightAccess;
        // Prevents instantiation unless accessible
        if (limelightAccess) {
            // Checks which robot to change address
            if (RobotPreferences.getInstance().getTeamNumber() == 930) {
                limelightCamera = new HttpCamera("limelight", "http://10.9.30.11:5801/stream.mjpg");
            } else {
                limelightCamera = new HttpCamera("limelight", "http://10.99.30.11:5801/stream.mjpg");
            }
            driverStationTab.add("LL", limelightCamera);
        } else {
            limelightCamera = null;
        }
    }

    /**
     * Toggles the accessibility for the flywheel group.
     */
    public void toggleFlywheelAccess() {
        flywheelAccess = !flywheelAccess;
    }

    /**
     * Toggles the accessibility for the endgame group.
     */
    public void toggleEndgameAccess() {
        endgameAccess = !endgameAccess;
    }

    /**
     * Toggles the accessibility for the miscellaneous group.
     */
    public void toggleMiscellaneousAccess() {
        miscellaneousAccess = !miscellaneousAccess;
    }

    // ------- Drive Tab -------\\

    /**
     * Set the intaking state on the Shuffleboard.
     * 
     * @param IntakeIndicator boolean state for the intake
     */
    public void putIntakeIndicator(boolean IntakeIndicator) {
        intakeIndicator = IntakeIndicator;
        if (intakeAccess) {
            intakingEntry.setBoolean(intakeIndicator);
        }
    }

    /**
     * Set the shooting state on the Shuffleboard.
     * 
     * @param ShootIndicator boolean state for the shooter
     */
    public void putShootIndicator(boolean ShootIndicator) {
        shootIndicator = ShootIndicator;
        if (flywheelAccess) {
            shootingEntry.setBoolean(shootIndicator);
        }
    }

    /**
     * Set the manual-mode state on the Shuffleboard.
     * 
     * @param ManualMode boolean state for the robot
     */
    public void putManualMode(boolean ManualMode) {
        manualMode = ManualMode;
        if (miscellaneousAccess) {
            manualModeEntry.setBoolean(manualMode);
        }
    }

    /**
     * Set the limelight data on the Shuffleboard.
     * 
     * @param DistanceFromTarget the calculated distance from the target
     */
    public void putDistanceFromTarget(double DistanceFromTarget) {
        distanceFromTarget = DistanceFromTarget;
        if (limelightAccess) {
            distanceFromTargetEntry.setNumber(distanceFromTarget);
        }
    }

    /**
     * Set the shot type state on the Shuffleboard.
     * 
     * @param ShotType enum state for the shooter
     */
    // TODO: find method for shot types
    public void putShotType(String ShotType) {
        shotType = ShotType;
        if (miscellaneousAccess) {
            shotTypeEntry.setString(shotType);
        }
    }

    /**
     * Set the hopper state on the Shuffleboard.
     * 
     */
    public void putHopperFeed() {

    }

    /**
     * Set the shooter speed state on the Shuffleboard.
     * 
     * @param ShooterUpToSpeed boolean state for the shooter speed
     */
    public void putShooterUpToSpeed(boolean ShooterUpToSpeed) {
        shooterUpToSpeed = ShooterUpToSpeed;
        if (flywheelAccess) {
            shooterUpToSpeedEntry.setBoolean(shooterUpToSpeed);
        }
    }

    /**
     * Set the flywheel speed on the Shuffleboard.
     * 
     * @param FlywheelSpeed speed of the flywheel
     */
    public void putFlywheelSpeed(double FlywheelSpeed) {
        flywheelSpeed = FlywheelSpeed;
        if (flywheelAccess) {
            flywheelSpeedEntry.setNumber(flywheelSpeed);
        }
    }

    /**
     * Set the flywheel voltage on the Shuffleboard.
     * 
     * @param FlywheelVoltage voltage of the flywheel motors
     */
    public void putFlywheelVoltage(double FlywheelVoltage) {
        flywheelVoltage = FlywheelVoltage;
        if (flywheelAccess) {
            flywheelVoltageEntry.setNumber(flywheelVoltage);
        }
    }

    /**
     * Set the flywheel control data on the Shuffleboard.
     * 
     * @param mVelError
     * @param mControlTol
     * @param mModelAcc
     * @param mEncodAcc
     * @param mMaxVoltage
     * @param mDtSeconds
     */
    public void putControlConfig(double mVelError, double mControlTol, double mModelAcc, double mEncodAcc,
            double mMaxVoltage, double mDtSeconds) {
        velError = mVelError;
        controlTol = mControlTol;
        modelAcc = mModelAcc;
        encodAcc = mEncodAcc;
        maxVoltage = mMaxVoltage;
        dtSeconds = mDtSeconds;
        if (flywheelAccess) {
            velErrorEntry.setNumber(velError);
            controlTolEntry.setNumber(controlTol);
            modelAccEntry.setNumber(modelAcc);
            encodAccEntry.setNumber(encodAcc);
            maxVoltageEntry.setNumber(maxVoltage);
            dtSecondsEntry.setNumber(dtSeconds);
        }

    }

    // ----- Testing & Debugging -----\\

    /**
     * Set the turret speed on the Shuffleboard.
     * 
     * @param TurretSpeed speed of the turret
     */
    public void putTurretSpeed(double TurretSpeed) {
        turretSpeed = TurretSpeed;
        if (turretAccess) {
            turretSpeedEntry.setNumber(turretSpeed);
        }
    }

    /**
     * Set the hopper speed on the Shuffleboard.
     * 
     * @param HopperSpeed speed of the hopper
     */
    public void putHopperSpeed(double HopperSpeed) {
        hopperSpeed = HopperSpeed;
        if (hopperAccess) {
            hopperSpeedEntry.setNumber(hopperSpeed);
        }
    }

    /**
     * Set the shooter angle state on the Shuffleboard.
     * 
     * @param ShooterAngle boolean state for the shooter position
     */
    public void putShooterAngle(boolean ShooterAngle) {
        shooterAngle = ShooterAngle;
        if (flywheelAccess) {
            shooterAngleEntry.setBoolean(shooterAngle);
        }
    }

    /**
     * Set the endgame encoder position on the Shuffleboard.
     * 
     * @param encoderPosition position of the endgame encoder
     */
    public void putEndgameEncoderPosition(double encoderPosition) {
        endgameEncoderPosition = encoderPosition;
        if (endgameAccess) {
            endgameEncoderPositionEntry.setDouble(endgameEncoderPosition);
        }
    }

    /**
     * Set the shooter speed on the Shuffleboard.
     * 
     * @param ShooterRPM speed of the shooter
     */
    public void putShooterRPM(double ShooterRPM) {
        shooterRPM = ShooterRPM;
        if (flywheelAccess) {
            shooterRPMEntry.setNumber(shooterRPM);
        }
    }

    /**
     * Set the turret encoder position on the Shuffleboard.
     * 
     * @param TurretEncoderPosition position of the turret encoder
     */
    public void putTurretEncoderPosition(double TurretEncoderPosition) {
        turretEncoderPosition = TurretEncoderPosition;
        if (turretAccess) {
            turretEncoderPositionEntry.setNumber(turretEncoderPosition);
        }
    }

    /**
     * Set the gyre yaw on the Shuffleboard.
     * 
     * @param GyroYaw yaw angle on the gyro
     */
    public void putGyroYaw(double GyroYaw) {
        gyroYaw = GyroYaw;
        if (miscellaneousAccess) {
            gyroYawEntry.setNumber(gyroYaw);
        }
    }

    // ----- Miscellaneous -----\\

    /**
     * Adds an option for auton selection
     * @param pathName name of the path added
     * @param autoCommand instance of the command being added
     */
    public void addAutonOptions(String pathName, CommandBase autoCommand) {
        autonChooser.addOption(pathName, autoCommand);
    }

    /**
     * Sets the default option for auton selection
     * @param pathName name of the path added
     * @param autoCommand instance of the command being added
     */
    public void setDefaultAutonOptions(String pathName, CommandBase autoCommand) {
        autonChooser.setDefaultOption(pathName, autoCommand);
    }

    /**
     * get the option selected on the Shuffleboard
     * @return the selected auton path
     */
    public Command getSelectedAutonPath() {
        return autonChooser.getSelected();
    }

    /**
     * Adds an additional Shuffleboard setup
     * @param settingName name for the added tab
     * @param settingCommand instance of the command being added
     */
    public void addShuffleboardOptions(String settingName, CommandBase settingCommand) {
        shuffleboardChooser.addOption(settingName, settingCommand);
    }

    /**
     * Sets the default setup
     * @param settingName name for the tab
     * @param settingCommand instance of the command being added
     */
    public void setDefaultShuffleboardOptions(String settingName, CommandBase settingCommand) {
        shuffleboardChooser.setDefaultOption(settingName, settingCommand);
    }

    /**
     * Get the selected setup from the Shuffleboard
     * @return the selected setup
     */
    public Command getSelectedShuffleboardOption() {
        return shuffleboardChooser.getSelected();
    }

    /**
     * Disables streaming of the limelight to the Shuffleboard.
     */
    public void disableLimelightStream() {
        limelightCamera = null;
    }
} // end of class Shuffleboard
