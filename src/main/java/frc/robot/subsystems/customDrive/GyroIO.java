package frc.robot.subsystems.customDrive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d roll = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;
        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation3d fullRotation = Rotation3d.kZero;
        public double rollVelocity = 0;
        public double pitchVelocity = 0;
        public double yawVelocity = 0;
        public double[] odometryRollTimestamps = new double[] {};
        public double[] odometryPitchTimestamps = new double[] {};
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryRollPositions = new Rotation2d[] {};
        public Rotation2d[] odometryPitchPositions = new Rotation2d[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {};
}
