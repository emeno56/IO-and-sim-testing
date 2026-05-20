package frc.robot.subsystems.customDrive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        //drive logging
        public boolean driveConnected = false;
        public double drivePosition = 0;
        public double driveStatorCurrent = 0;
        public double driveSupplyCurrent = 0;
        public double driveTorqueCurrent = 0;
        public double driveVelocityRadPerSec = 0;
        public double driveSupplyVolts = 0;
        public double driveMotorVolts = 0;
        public double driveTemperature = 0;

        //steer logging
        public boolean steerConnected = false;
        public boolean absoluteEncoderConnected = false;
        public Rotation2d steerPosition = Rotation2d.kZero;
        public Rotation2d absolutePosition = Rotation2d.kZero;
        public double steerStatorCurrent = 0;
        public double steerSupplyCurrent = 0;
        public double steerTorqueCurrent = 0;
        public double steerVelocityRadPerSec = 0;
        public double steerSupplyVolts = 0;
        public double steerMotorVolts = 0;
        public double steerTemperature = 0;

        //module logging
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Update the set of logged inputs */
    public default void updateInputs(ModuleIOInputs io) {}

    /** Set the drive at the requested open loop value */
    public default void setDriveOpenLoop(double output) {}

    /** Set the steer at the requested open loop value */
    public default void setSteerOpenLoop(double output) {}

    /** Set the drive to the requested velocity */
    public default void setDriveVelocity(double driveVelocityRadPerSec) {}

    /** Set the steer to the requested position */
    public default void setSteerPosition(Rotation2d pos) {}
}
