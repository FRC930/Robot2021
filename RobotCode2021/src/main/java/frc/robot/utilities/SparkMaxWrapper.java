package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;

/**
 * <h4>SparkMaxWrapper</h4> Assists in simulation of Spark Max objects via a
 * wrapper class.
 */
public class SparkMaxWrapper extends CANSparkMax {
    private SimDouble m_simSpeed;
    private SimDevice m_simSparkMax;

    /**
     * Creates the simulated Spark Max object
     * 
     * @param deviceID ID location for the motor
     * @param type     enum MototType declares the brushed type of the motor being
     *                 used. kBrushed is 775s and Cim motors, kBrushless are NEOs
     */
    public SparkMaxWrapper(int deviceID, MotorType type) {
        // calls the parent constructor for a normal Spark Max
        super(deviceID, type);

        m_simSparkMax = SimDevice.create("SparkMax", deviceID);
        // Checks for valid simulation object
        // TODO: find non-depreciated method replacement
        if (m_simSparkMax != null) {
            m_simSpeed = m_simSparkMax.createDouble("speed", false, 0.0);
        }
    }

    /**
     * @return the simulation speed of the Spark Max
     */
    @Override
    public double get() {
        // Checks for valid simulation object
        if (m_simSparkMax != null) {
            return m_simSpeed.get();
        }
        return super.get();
    }

    /**
     * @param speed the desired simulation speed
     */
    @Override
    public void set(double speed) {
        // Checks for valid simulation object
        if (m_simSparkMax != null) {
            m_simSpeed.set(speed);
        } else {
            super.set(speed);
        }
    }

    /**
     * @param outputVolts simulated voltage output for the Spark Max
     */
    @Override
    public void setVoltage(double outputVolts) { // For simulation purposes, we are expecting that the battery voltage
                                                 // stays constant.
        // Checks for valid simulation object. Goes to parent if not.
        if (m_simSparkMax != null) {
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }
}