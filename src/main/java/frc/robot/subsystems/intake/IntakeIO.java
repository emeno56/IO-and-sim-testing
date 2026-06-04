package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean leftIsConnected = false;
        public double leftPosition = 0;
        public double leftStatorCurrent = 0;
        public double leftSupplyCurrent = 0;
        public double leftTorqueCurrent = 0;
        public double leftVelocityRadPerSec = 0;
        public double leftSupplyVolts = 0;
        public double leftMotorVolts = 0;
        public double leftTemperature = 0;

        public boolean rightIsConnected = false;
        public double rightPosition = 0;
        public double rightStatorCurrent = 0;
        public double rightSupplyCurrent = 0;
        public double rightTorqueCurrent = 0;
        public double rightVelocityRadPerSec = 0;
        public double rightSupplyVolts = 0;
        public double rightMotorVolts = 0;
        public double rightTemperature = 0;

        public boolean isExtended = false;
        public double extendDistance = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakePosition(double inches) {}
}
