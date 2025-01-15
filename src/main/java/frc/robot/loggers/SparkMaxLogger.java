package frc.robot.loggers;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax>{
    public SparkMaxLogger() {
        super(SparkMax.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkMax sparkMax) {
        backend.log("SparkMax CAN ID", sparkMax.getDeviceId());
        backend.log("Requested Speed (%)", sparkMax.get());
        backend.log("Applied output (%)", sparkMax.getAppliedOutput());
        backend.log("Input Voltage (V)", sparkMax.getBusVoltage());
        backend.log("Motor Temperature (C)", sparkMax.getMotorTemperature());
        backend.log("Output Current (A)", sparkMax.getOutputCurrent());
    }
    
}
