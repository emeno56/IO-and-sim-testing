package frc.robot.subsystems.customDrive;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2Real extends GyroIOPigeon2 {
    // make odometry queues
    private final Queue<Double> rollPosQueue;
    private final Queue<Double> rollTimestampQueue;
    private final Queue<Double> pitchPosQueue;
    private final Queue<Double> pitchTimestampQueue;
    private final Queue<Double> yawPosQueue;
    private final Queue<Double> yawTimestampQueue;
    
    public GyroIOPigeon2Real() {
        super();
        //set queues
        rollPosQueue = PhoenixOdometryThread.getInstance().registerSignal(roll.clone());
        pitchPosQueue = PhoenixOdometryThread.getInstance().registerSignal(pitch.clone());
        yawPosQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());

        rollTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        pitchTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        super.updateInputs(inputs);
        
        // do queue stuff
        inputs.odometryRollTimestamps = 
            rollTimestampQueue.stream().mapToDouble((Double val) -> val).toArray();
        inputs.odometryPitchTimestamps = 
            pitchTimestampQueue.stream().mapToDouble((Double val) -> val).toArray();
        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double val) -> val).toArray();

        inputs.odometryRollPositions = rollPosQueue.stream()
            .map((Double val) -> Rotation2d.fromDegrees(val))
            .toArray(Rotation2d[]::new);
        inputs.odometryPitchPositions = pitchPosQueue.stream()
            .map((Double val) -> Rotation2d.fromDegrees(val))
            .toArray(Rotation2d[]::new);
        inputs.odometryYawPositions = yawPosQueue.stream()
            .map((Double val) -> Rotation2d.fromDegrees(val))
            .toArray(Rotation2d[]::new);

        rollTimestampQueue.clear();
        pitchTimestampQueue.clear();
        yawTimestampQueue.clear();
        rollPosQueue.clear();
        pitchPosQueue.clear();
        yawPosQueue.clear();
    }
}
