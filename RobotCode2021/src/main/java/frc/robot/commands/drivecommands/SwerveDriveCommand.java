package frc.robot.commands.drivecommands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.logging.*;

public class SwerveDriveCommand extends CommandBase{
    
  //-------- CONSTANTS --------\\

  //-------- DECLARATIONS --------\\

  private final DriveSubsystem sdSubsystem;
  private final Joystick driveStick;
  private int driverLeftX;
  private int driverLeftY;
  private int driverRightX;

  private static final Logger logger = Logger.getLogger(DriveCommand.class.getName());

  //-------- CONSTRUCTOR --------\\

  public SwerveDriveCommand(DriveSubsystem _sdSubsystem, Joystick _driverStick, int _driverLeftX, int _driverLeftY, int _driverRightX) {
    sdSubsystem = _sdSubsystem;
    driveStick = _driverStick;
    setSwerveAxis(_driverLeftX, _driverLeftY, _driverRightX);
    addRequirements(sdSubsystem);  // Use addRequirements() here to declare subsystem dependencies.
  } 

  //-------- COMMANDBASE METHODS --------\\

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
  }

  @Override   // Called every time the scheduler runs while the command is scheduled.
  public void execute() {  
    // Send left and right joystick axis to the run method
    run(driveStick.getRawAxis(driverLeftX), driveStick.getRawAxis(driverLeftY), driveStick.getRawAxis(driverRightX));
  }
  
  @Override   // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
  }

  //-------- METHODS --------\\

  // Sets up three driver axis in accordance with 2 joysticks
  //                            left              left              right          joystick IDs
  public void setSwerveAxis(int _driverLeftX, int _driverLeftY, int _driverRightX) {
    driverLeftX = _driverLeftX;
    driverLeftY = _driverLeftY;
    driverRightX = _driverRightX;
    
  }
  
  // Swerve Drive Logic
  private void run(double leftX, double leftY, double rightX) {

    // TODO: Create deadband constants for each axis
    // Set value to zero if under deadband limit
    if (Math.abs(leftX) < 0.15) {
        logger.log(Level.INFO, "leftX < Left Joystick's X-Axis Deadband");
        leftX = 0;
    }
    if (Math.abs(leftY) < 0.15) {
        logger.log(Level.INFO, "leftY < Left Joystick's Y-Axis Deadband");
        leftY = 0;
    }
    if (Math.abs(rightX) < 0.1) {
        logger.log(Level.INFO, "rightX < Right Joystick's Y-Axis Deadband");
        rightX = 0;
    }

    // Cube values to create smoother movement
    leftX = -Math.pow(leftX, 1);
    leftY = Math.pow(leftY, 1); // reverse orientation of the forward and back because players sit opposite of robot
    rightX = -Math.pow(rightX, 1);
    
    SmartDashboard.putNumber("Drive Leftx", leftX);
    SmartDashboard.putNumber("Drive Lefty", leftY);
    SmartDashboard.putNumber("Drive Rightx", rightX);
  
    // Sends resulting values to SwerveDriveSubsystem for the values to be translated into physical movement
    sdSubsystem.swerveDrive(leftX, leftY, rightX);

  } // End of run() method

} //End of class SwerveDriveCommand
