/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.shootercommands.flywheelcommands;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.utilities.ShooterControl;
import frc.robot.utilities.ShuffleboardUtility;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;



//-------- COMMAND CLASS --------\\

public class DefaultFlywheelCommand extends CommandBase {

    //-------- DECLARATIONS --------\\

    private FlywheelSubsystem m_FlywheelSubsystem;

    private final double neoEncoderRate = 42;
    private final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2000.0);
    public final double kFlywheelMomentOfInertia = 0.00094610399;  //(new from CAD) 614.207709 kg * m^2
    private final double kFlywheelGearing = 1.6666666667;   
                                                //Try> 34.406481 | 614.207709 | 24190.1501335 | 
                                                


    private double mVelError = 8.0;
    private double mControlTol = 12.0;
    private double mModelAcc = 3.0;
    private double mEncodAcc = 0.01;
    private double mMaxVoltage = 12.0;
    private double mDtSeconds = 0.020;
    private LinearSystem<N1, N1, N1> m_flywheelPlant;
    private KalmanFilter<N1, N1, N1> m_observer;
    private LinearQuadraticRegulator<N1, N1, N1> m_controller;
    private LinearSystemLoop<N1, N1, N1> m_loop;
    
    private ShuffleboardUtility ModelConfig = ShuffleboardUtility.getInstance();

    //-------- CONSTRUCTOR --------\\
    public DefaultFlywheelCommand(FlywheelSubsystem flywheelSubsystem) {
        m_FlywheelSubsystem = flywheelSubsystem;
        addRequirements(m_FlywheelSubsystem);
        m_flywheelPlant =
          LinearSystemId.createFlywheelSystem(
          DCMotor.getNEO(2), kFlywheelMomentOfInertia, kFlywheelGearing);

        m_controller =
        new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(mVelError), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(mControlTol), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(mModelAcc), // How accurate we think our model is
            VecBuilder.fill(mEncodAcc), // How accurate we think our encoder
            // data is
            0.020);

        m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, mMaxVoltage, mDtSeconds);

    }
    
    public DefaultFlywheelCommand(FlywheelSubsystem flywheelSubsystem, double velError, double controlTol, double modelAcc, double encodAcc, double maxVoltage, double dtSeconds) {
        mVelError = velError;
        mControlTol = controlTol;
        mEncodAcc = encodAcc;
        mModelAcc = modelAcc;
        mMaxVoltage = maxVoltage;
        mDtSeconds = dtSeconds;
        m_FlywheelSubsystem = flywheelSubsystem;
        ModelConfig.putControlConfig(mVelError, mControlTol, mModelAcc, mEncodAcc, mMaxVoltage, mDtSeconds);
        addRequirements(m_FlywheelSubsystem);
        m_flywheelPlant =
          LinearSystemId.createFlywheelSystem(
          DCMotor.getNEO(2), kFlywheelMomentOfInertia, kFlywheelGearing);

        m_controller =
        new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(mVelError), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(mControlTol), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(mModelAcc), // How accurate we think our model is
            VecBuilder.fill(mEncodAcc), // How accurate we think our encoder
            // data is
            0.020);

        m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, mMaxVoltage, mDtSeconds);

    }


    //-------- COMMANDBASE METHODS --------\\

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
        // We just pressed the trigger, so let's set our next reference
        m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    
  
      // Correct our Kalman filter's state vector estimate with encoder data.
      m_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_FlywheelSubsystem.getSpeed())));

      // Update our LQR to generate new voltage commands and use the voltages to predict the next
      // state with out Kalman filter.
      m_loop.predict(0.020);
  
      // Send the new calculated voltage to the motors.
      // voltage = duty cycle * battery voltage, so
      // duty cycle = voltage / battery voltage
      double nextVoltage = m_loop.getU(0);
      m_FlywheelSubsystem.setVoltage(nextVoltage); 
      ModelConfig.putFlywheelVoltage(m_FlywheelSubsystem.getVoltage());
      ModelConfig.putFlywheelSpeed(m_FlywheelSubsystem.getSpeed());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_loop.setNextR(VecBuilder.fill(0.0));
         // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(neoEncoderRate));
    
        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        m_loop.predict(0.020);
    
        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0);
        m_FlywheelSubsystem.setVoltage(nextVoltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

} // end of class RunDefaultFlywheelCommand
