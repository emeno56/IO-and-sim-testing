package frc.robot.subsystems.customDrive;

import java.util.Queue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class ModuleIOKrakenReal extends ModuleIOKraken{
    //timestamps
    private final Queue<Double> timeStampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steerPositionQueue;

    public ModuleIOKrakenReal(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        super(constants);

        //Queueing init
        timeStampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerPosition.clone());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);
        //Update timestamps
        inputs.odometryTimestamps = timeStampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
        inputs.odometryTurnPositions =
        steerPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
        timeStampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }
}
