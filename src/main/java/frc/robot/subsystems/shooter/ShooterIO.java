package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean motor1IsConnected = false;
        public double motor1Position = 0;
        public double motor1StatorCurrent = 0;
        public double motor1SupplyCurrent = 0;
        public double motor1TorqueCurrent = 0;
        public double motor1VelocityRotPerSec = 0;
        public double motor1SupplyVolts = 0;
        public double motor1MotorVolts = 0;
        public double motor1Temperature = 0;

        public boolean motor2IsConnected = false;
        public double motor2Position = 0;
        public double motor2StatorCurrent = 0;
        public double motor2SupplyCurrent = 0;
        public double motor2TorqueCurrent = 0;
        public double motor2VelocityRotPerSec = 0;
        public double motor2SupplyVolts = 0;
        public double motor2MotorVolts = 0;
        public double motor2Temperature = 0;

        public boolean motor3IsConnected = false;
        public double motor3Position = 0;
        public double motor3StatorCurrent = 0;
        public double motor3SupplyCurrent = 0;
        public double motor3TorqueCurrent = 0;
        public double motor3VelocityRotPerSec = 0;
        public double motor3SupplyVolts = 0;
        public double motor3MotorVolts = 0;
        public double motor3Temperature = 0;

        public boolean motor4IsConnected = false;
        public double motor4Position = 0;
        public double motor4StatorCurrent = 0;
        public double motor4SupplyCurrent = 0;
        public double motor4TorqueCurrent = 0;
        public double motor4VelocityRotPerSec = 0;
        public double motor4SupplyVolts = 0;
        public double motor4MotorVolts = 0;
        public double motor4Temperature = 0;

        public boolean hoodIsConnected = false;
        public double hoodPosition = 0;
        public double hoodStatorCurrent = 0;
        public double hoodSupplyCurrent = 0;
        public double hoodTorqueCurrent = 0;
        public double hoodVelocityRotPerSec = 0;
        public double hoodSupplyVolts = 0;
        public double hoodMotorVolts = 0;
        public double hoodTemperature = 0;

        public double fuelExitVelocityMPS = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    
    public default void setFuelExitVelocity(double mps) {}

    public default void setHoodAngle(double degrees) {}

    public default void setShooterState(double mps, double degrees) {}

    public default void stop() {}
}
