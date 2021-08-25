package frc.robot.commands.drivecommands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.logging.*;

import static frc.robot.subsystems.DriveSubsystem.DRIVER_LEFT_AXIS_X_DEADBAND;
import static frc.robot.subsystems.DriveSubsystem.DRIVER_LEFT_AXIS_Y_DEADBAND;
import static frc.robot.subsystems.DriveSubsystem.DRIVER_RIGHT_AXIS_X_DEADBAND;

/**
 * <h3>SwerveDriveCommand</h3>
 * 
 * This class controls the swerve drive that is used on the competition robot
 */
public class SwerveDriveCommand extends CommandBase {

    // -------- CONSTANTS --------\\

    // -------- DECLARATIONS --------\\

    private final DriveSubsystem sdSubsystem;
    private final Joystick driveStick;
    private int driverLeftX;
    private int driverLeftY;
    private int driverRightX;

    private static final Logger logger = Logger.getLogger(SwerveDriveCommand.class.getName());

    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>SwerveDriveCommand</h3>
     * 
     * Instantiates a {@link frc.robot.commands.drivecommands.SwerveDriveCommand
     * SwerveDriveCommand} for controlling the robot's movement
     * 
     * @param sdSubsystem  the instance of the drive subsystem that will be used
     * @param driverStick  which joystick to use as the driver stick
     * @param driverLeftX  the ID of the <i>x</i> axis of the left stick
     * @param driverLeftY  the ID of the <i>y</i> axis of the left stick
     * @param driverRightX the ID of the <i>x</i> axis of the right stick
     */
    public SwerveDriveCommand(DriveSubsystem sdSubsystem, Joystick driverStick, int driverLeftX, int driverLeftY,
            int driverRightX) {
        this.sdSubsystem = sdSubsystem;
        this.driveStick = driverStick;
        setSwerveAxis(driverLeftX, driverLeftY, driverRightX);
        addRequirements(sdSubsystem); // Use addRequirements() here to declare subsystem dependencies.
    }

    // -------- COMMANDBASE METHODS --------\\

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Send left and right joystick axis to the run method
        run(driveStick.getRawAxis(driverLeftX), driveStick.getRawAxis(driverLeftY),
                driveStick.getRawAxis(driverRightX));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // -------- METHODS --------\\

    /**
     * <h3>setSwerveAxis</h3>
     * 
     * This method assigns the joystick axes for the driver
     * 
     * @param driverLeftX  the left <i>x</i> axis
     * @param driverLeftY  the left <i>y</i> axis
     * @param driverRightX the right <i>x</i> axis
     */
    private void setSwerveAxis(int driverLeftX, int driverLeftY, int driverRightX) {
        this.driverLeftX = driverLeftX;
        this.driverLeftY = driverLeftY;
        this.driverRightX = driverRightX;

    }

    // Swerve Drive Logic
    private void run(double leftX, double leftY, double rightX) {
        /**
         * Deadband Logic
         * 
         * Uses the constants defined in DriveSubsystem to check if the joysticks are
         * within the deadband
         * 
         * If they are, the values are set to zero
         */
        if (Math.abs(leftX) < DRIVER_LEFT_AXIS_X_DEADBAND) {
            logger.log(Level.INFO, "Swerve Deadband Logic: leftX within deadband. Value set to 0");
            leftX = 0;
        }
        if (Math.abs(leftY) < DRIVER_LEFT_AXIS_Y_DEADBAND) {
            logger.log(Level.INFO, "Swerve Deadband Logic: leftY within deadband. Value set to 0");
            leftY = 0;
        }
        if (Math.abs(rightX) < DRIVER_RIGHT_AXIS_X_DEADBAND) {
            logger.log(Level.INFO, "Swerve Deadband Logic: rightX within deadband. Value set to 0");
            rightX = 0;
        }

        /**
         * This is where we could cube the values of the joysticks to allow for a
         * smoother transition from stop to full speed
         * 
         * <pre>
         * 
         * Linear function:
         *  _________
         * |       / |
         * |      /  |
         * |     /   |
         * |    /    |
         * |   /     |
         * |  /      |
         * |_/_______|
         * 
         * Cubic function: 
         *  _________
         * |        ||
         * |        ||
         * |        /|
         * |        ||
         * |       / |
         * |    _⸗̅   |
         * |_--ˉ     |
         *  ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅
         * </pre>
         */
        // Refine power inputs by increasing function power
        leftX = Math.pow(leftX, 1);
        leftY = Math.pow(leftY, 1);
        // Reverse x input
        rightX = -Math.pow(rightX, 1);

        // Put the modified numbers on the SmartDashboard for the drive team
        SmartDashboard.putNumber("Drive LeftX", leftX);
        SmartDashboard.putNumber("Drive LeftY", leftY);
        SmartDashboard.putNumber("Drive RightX", rightX);

        // Send the results to SwerveDriveSubsystem for execution
        sdSubsystem.swerveDrive(leftX, leftY, rightX);

    } // End of run() method

} // End of class SwerveDriveCommand
