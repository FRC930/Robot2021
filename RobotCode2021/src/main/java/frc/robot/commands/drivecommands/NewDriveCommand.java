package frc.robot.commands.drivecommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewDriveSubsystem;

public class NewDriveCommand extends CommandBase {
    private final NewDriveSubsystem m_driveSubsystem;
    private final Joystick m_driverController;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;
    private final DoubleSupplier rotation;

    public NewDriveCommand(NewDriveSubsystem driveSubsystem, Joystick driverController, int driverLeftX,
            int driverLeftY, int driverRightX) {
        m_driveSubsystem = driveSubsystem;
        m_driverController = driverController;
        translationX = () -> -modifyAxis(m_driverController.getRawAxis(driverLeftX))
                * NewDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        translationY = () -> -modifyAxis(m_driverController.getRawAxis(driverLeftY))
                * NewDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        rotation = () -> -modifyAxis(m_driverController.getRawAxis(driverRightX))
                * NewDriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationX.getAsDouble(),
                translationY.getAsDouble(), rotation.getAsDouble(), m_driveSubsystem.getGyroRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.05);

        // Square axis
        value = Math.copySign(value * value, value);

        return value;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
}
