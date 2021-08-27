/**
 * This class simulates a drivetrain for desktop simulation of the robot.
 */
package frc.robot.utilities;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

@SuppressWarnings("PMD.TooManyFields")

//-------- CLASS --------\\
/**
 * TODO: figure out how to use this 
 * TODO: comment and describe every method
 * this class simulates the drive train for the simulation robot
 */
public class SimulatedDrivetrain {


  //-------- CONSTANTS --------\\

  public static final double KMAXSPEED = 3.0;
  // 3 meters per second
  public static final double KMAXANGULARSPEED = Math.PI;

  private static final double KTRACKWIDTH = 0.381 * 2;
  private static final double KWHEELRADIUS = 0.0508;
  private static final int KENCODERRESOLUTION = -4096;

  //-------Simulated Hardware-------\\
  private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
  private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
  private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
  private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);

  private final SpeedControllerGroup m_leftGroup =
      new SpeedControllerGroup(m_leftLeader, m_leftFollower);
  private final SpeedControllerGroup m_rightGroup =
      new SpeedControllerGroup(m_rightLeader, m_rightFollower);

  private final Encoder m_leftEncoder = new Encoder(6, 7);
  private final Encoder m_rightEncoder = new Encoder(8, 9);

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(KTRACKWIDTH);
  private final DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(m_gyro.getRotation2d());

  // Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation objects help us simulate our robot
  private final AnalogGyroSim m_gyroSim  = new AnalogGyroSim(m_gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(
          m_drivetrainSystem, DCMotor.getCIM(2), 8, KTRACKWIDTH, KWHEELRADIUS, null);

  
  //-------- CONSTRUCTOR --------\\
  /**
   * this constructs the class
   */
  public SimulatedDrivetrain() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * KWHEELRADIUS / KENCODERRESOLUTION);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * KWHEELRADIUS /KENCODERRESOLUTION);
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_rightGroup.setInverted(true);
    SmartDashboard.putData("Field", m_fieldSim);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftLeader.get() * RobotController.getInputVoltage(),
        -m_rightLeader.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }
}