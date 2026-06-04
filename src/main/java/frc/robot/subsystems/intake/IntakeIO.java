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

        public boolean rollerIsConnected = false;
        public double rollerPosition = 0;
        public double rollerStatorCurrent = 0;
        public double rollerSupplyCurrent = 0;
        public double rollerTorqueCurrent = 0;
        public double rollerVelocityRadPerSec = 0;
        public double rollerSupplyVolts = 0;
        public double rollerMotorVolts = 0;
        public double rollerTemperature = 0;

        public boolean isRunning = false;
        public double extendDistance = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeState(double inches, double rotPerSec) {}

    public default void deployIntake() {}

    public default void stopIntake() {}

    public default void setIntakeDistance(double inches) {}

    public default void setRollerSpeed(double rotPerSec) {}
}