/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.shootercommands.flywheelcommands;

//import java.util.logging.*;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;

//-------- COMMAND CLASS --------\\

public class DefaultFlywheelCommand extends CommandBase {
  
  // -------- DECLARATIONS --------\\

  //this is the private object representing the subsystem
  private FlywheelSubsystem m_FlywheelSubsystem;

  private double kSpinupRadPerSec; 
  public final double KFLYWHEELMOMENTOFINERTIA = 0.00094610399; 
  private final double KFLYWHEELGEARING = 1.0;
  
  // Velocity Error - Velocity error tolerance, in radians per second. Decrease
  // this to more heavily penalize state excursion, or make the controller behave
  // more
  // aggressively.
  private double MVELERROR = 8.0;
  // Control effort (voltage) tolerance. Decrease this to more
  // heavily penalize control effort, or make the controller less aggressive. 12
  // is a good
  // starting point because that is the (approximate) maximum voltage of a
  // battery.
  private double MCONTROLTOL = 12.0;
  // Model Accuracy - How accurate we think our model is
  private double MMODELACC = 3.0;
  // Encoder Accuracy - How accurate we think our encoder is
  private double MENCODACC = 0.01;
  // Max Voltage - the max voltage , used for our loop
  private double MMAXVOLTAGE = 12.0;
  // dtSeconds - time between loops, sync up with TimedRobot
  private double MDTSECONDS = 0.020;

  private LinearSystem<N1, N1, N1> m_flywheelPlant;
  private KalmanFilter<N1, N1, N1> m_observer;
  private LinearQuadraticRegulator<N1, N1, N1> m_controller;
  private LinearSystemLoop<N1, N1, N1> m_loop;

  private ShuffleboardUtility shuffleboard = ShuffleboardUtility.getInstance();
  //private static final Logger logger = Logger.getLogger(DefaultFlywheelCommand.class.getName());

  // -------- CONSTRUCTOR --------\\
  /**
   * this constucts the class
   * @param flywheelSubsystem controls the flywheel
   */
  public DefaultFlywheelCommand(FlywheelSubsystem flywheelSubsystem) {

    //this code has been implamented from the Wpilib control base system
    m_FlywheelSubsystem = flywheelSubsystem;
    addRequirements(m_FlywheelSubsystem);
    m_flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), KFLYWHEELMOMENTOFINERTIA,
        KFLYWHEELGEARING);

    m_controller = new LinearQuadraticRegulator<>(m_flywheelPlant, 
        VecBuilder.fill(MVELERROR), // qelms.
        VecBuilder.fill(MCONTROLTOL), // relms.
        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

    m_observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), m_flywheelPlant, 
        VecBuilder.fill(MMODELACC), 
        VecBuilder.fill(MENCODACC), 
        // data is
        0.020);

    m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, MMAXVOLTAGE, MDTSECONDS);

    kSpinupRadPerSec = m_FlywheelSubsystem.getRadiansPerSecond();
  }

  // Constructor (the second one. m-modifiable)
  /**
   * this constucts the class, allows you to change attributes
   * TODO: refactor to use in other constructer
   * @param flywheelSubsystem
   * @param velError
   * @param controlTol
   * @param modelAcc
   * @param encodAcc
   * @param maxVoltage
   * @param dtSeconds
   */
  public DefaultFlywheelCommand(FlywheelSubsystem flywheelSubsystem, double velError, double controlTol,
      double modelAcc, double encodAcc, double maxVoltage, double dtSeconds) {
      
        MVELERROR = velError;
        MCONTROLTOL = controlTol;
    MENCODACC = encodAcc;
    MMODELACC = modelAcc;
    MMAXVOLTAGE = maxVoltage;
    MDTSECONDS = dtSeconds;
    m_FlywheelSubsystem = flywheelSubsystem;

    //shuffleboard.putControlConfig(MVELERROR, MCONTROLTOL, MMODELACC, MENCODACC, MMAXVOLTAGE, MDTSECONDS);
    addRequirements(m_FlywheelSubsystem);
    m_flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), KFLYWHEELMOMENTOFINERTIA,
        KFLYWHEELGEARING);

    m_controller = new LinearQuadraticRegulator<>(m_flywheelPlant,
      	VecBuilder.fill(MVELERROR), // qelms
        VecBuilder.fill(MCONTROLTOL), // relms 
        0.020); // Nominal time between loops. 0.020 for TimedRobot

    m_observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), m_flywheelPlant, VecBuilder.fill(MMODELACC), 
        VecBuilder.fill(MENCODACC), 
        // data is
        0.020);

    m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, MMAXVOLTAGE, MDTSECONDS);
  }

  // -------- COMMANDBASE METHODS --------\\

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kSpinupRadPerSec = m_FlywheelSubsystem.getRadiansPerSecond();
    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a PID controller.
    // We just pressed the trigger, so let's set our next reference
     m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));

    // Correct our Kalman filter's state vector estimate with encoder data.
     m_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_FlywheelSubsystem.getSpeed())));
      //System.out.println(m_FlywheelSubsystem.getSpeed());
    /// Update our LQR to generate new voltage commands and use the voltages to
    // predict the next state with out Kalman filter.
     m_loop.predict(MDTSECONDS);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so duty cycle = voltage / battery voltage
     double nextVoltage = m_loop.getU(0);
     m_FlywheelSubsystem.setVoltage(nextVoltage);
    shuffleboard.putFlywheelVoltage(m_FlywheelSubsystem.getVoltage());
    shuffleboard.putFlywheelSpeed(m_FlywheelSubsystem.getSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

} // end of class RunDefaultFlywheelCommand
