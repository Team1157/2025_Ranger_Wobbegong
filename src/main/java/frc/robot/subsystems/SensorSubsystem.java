package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that interfaces with the Gravity 7/24 Industrial Analog pH Meter Kit.
 * Reads pH values from an analog input and provides logged data for monitoring the
 * one task with the waterbottle
 */
public class SensorSubsystem extends SubsystemBase {
    // Hardware
    private final AnalogInput m_phSensor;

    // pH calculation constants
    private static final double VOLTS_TO_PH_SLOPE = -0.0172; // This value may need calibration
    private static final double PH_CALIBRATION_OFFSET = 7.0; // Neutral pH at specified voltage
    private static final double CALIBRATION_VOLTAGE = 2.5; // Voltage at neutral pH (needs calibration_!!!!!)
    
    // Filtering
    private double m_filteredPhValue = 7.0; // Start with neutral pH
    private static final double FILTER_ALPHA = 0.1; // Filtering constant higher is smooooth
    
    // NetworkTables publishers
    private final DoublePublisher m_rawVoltagePublisher;
    private final DoublePublisher m_phValuePublisher;
    private final DoublePublisher m_filteredPhValuePublisher;
    
    /**
     * 
     * @param pHSensorAnalogPort The analog input port the pH sensor is connected to
     */
    public SensorSubsystem(int pHSensorAnalogPort) {
        // Initialize hardware
        m_phSensor = new AnalogInput(pHSensorAnalogPort);
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setFPS(30);
        camera.setResolution(640, 40);
        camera.setPixelFormat(PixelFormat.kMJPEG);
        // Configure the analog input for better readings
        m_phSensor.setAverageBits(4); // Average 16 samples for noise reduction
        
        // Initialize NetworkTable publishers
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        m_rawVoltagePublisher = nt.getDoubleTopic("/Sensors/pH/RawVoltage").publish();
        m_phValuePublisher = nt.getDoubleTopic("/Sensors/pH/Value").publish();
        m_filteredPhValuePublisher = nt.getDoubleTopic("/Sensors/pH/FilteredValue").publish();
        
        // Log that we've initialized
        Logger.recordOutput("PhSensor/Initialized", true);
    }
    
    @Override
    public void periodic() {
        // Get the raw voltage from the sensor (0-5V range mapped to 0-5000mV)
        double rawVoltage = m_phSensor.getVoltage() * 1000; // Convert to millivolts
        
        double phValue = convertVoltageToPh(rawVoltage);
        
        // Apply low-pass filter to smooth readings (I still dont trust the sensor)
        m_filteredPhValue = (FILTER_ALPHA * phValue) + ((1 - FILTER_ALPHA) * m_filteredPhValue);
        
        // Publish values to NetworkTables
        m_rawVoltagePublisher.set(rawVoltage);
        m_phValuePublisher.set(phValue);
        m_filteredPhValuePublisher.set(m_filteredPhValue);
        
        // Log values using AdvantageKit
        Logger.recordOutput("PhSensor/RawVoltage", rawVoltage);
        Logger.recordOutput("PhSensor/PhValue", phValue);
        Logger.recordOutput("PhSensor/FilteredPhValue", m_filteredPhValue);
    }
    
    /**
     * This formula may need calibration for our specific sensor, idk if the docs are right
     * 
     * @param millivolts The raw voltage reading in millivolts
     * @return The calculated pH value
     */
    private double convertVoltageToPh(double millivolts) {
        // Based on the Arduino example: voltage = analogRead(PH_PIN)/1024.0*5000
        // We've already converted to millivolts in the periodic method
        
        // Convert millivolts to pH using a linear relationship
        // pH = offset + slope * (voltage - calibration_voltage)
        return PH_CALIBRATION_OFFSET + VOLTS_TO_PH_SLOPE * (millivolts - CALIBRATION_VOLTAGE * 1000);
    }
    
    /**
     * 
     * @return The current filtered pH reading
     */
    public double getPh() {
        return m_filteredPhValue;
    }
    
    /**
     * 
     * @return The raw voltage reading from the sensor in millivolts
     */
    public double getRawMillivolts() {
        return m_phSensor.getVoltage() * 1000;
    }
    
    /**
     * 
     * @param knownPh The known pH value of the calibration solution
     */
    public void calibrate(double knownPh) {
        double currentVoltage = m_phSensor.getVoltage() * 1000;
        Logger.recordOutput("PhSensor/CalibrationVoltage", currentVoltage);
        Logger.recordOutput("PhSensor/CalibrationPh", knownPh);
    }
    
    /**
     * Runs a test routine for the pH sensor cause no way its good ootb
     */
    public void testPhSensor() {
        double voltage = m_phSensor.getVoltage() * 1000;
        double ph = convertVoltageToPh(voltage);
        
        System.out.println("pH Sensor Test:");
        System.out.println("Raw Voltage: " + voltage + " mV");
        System.out.println("Calculated pH: " + ph);
        System.out.println("Filtered pH: " + m_filteredPhValue);
    }
}