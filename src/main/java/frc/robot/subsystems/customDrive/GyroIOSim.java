package frc.robot.subsystems.customDrive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.PhoenixUtil;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSim;

    public GyroIOSim(GyroSimulation sim) {
        this.gyroSim = sim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yaw = gyroSim.getGyroReading();
        inputs.yawVelocity = gyroSim.getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.odometryYawPositions = gyroSim.getCachedGyroReadings();
        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.fullRotation = new Rotation3d(inputs.yaw);
    }
}
