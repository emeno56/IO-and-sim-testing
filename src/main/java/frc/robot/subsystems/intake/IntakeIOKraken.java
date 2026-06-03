package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOKraken implements IntakeIO{
    protected final TalonFX leftMotor;
    protected final TalonFX rightMotor;

    //control requests
    protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0).withEnableFOC(true);
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);

    protected final StatusSignal<Angle> leftPosition;
    protected final StatusSignal<Current> leftStatorCurrent;
    protected final StatusSignal<Current> leftSupplyCurrent;
    protected final StatusSignal<Current> leftTorqueCurrent;
    protected final StatusSignal<AngularVelocity> leftVelocity;
    protected final StatusSignal<Voltage> leftSupplyVoltage;
    protected final StatusSignal<Voltage> leftVoltage;
    protected final StatusSignal<Temperature> leftTemperature;

    protected final StatusSignal<Angle> rightPosition;
    protected final StatusSignal<Current> rightStatorCurrent;
    protected final StatusSignal<Current> rightSupplyCurrent;
    protected final StatusSignal<Current> rightTorqueCurrent;
    protected final StatusSignal<AngularVelocity> rightVelocity;
    protected final StatusSignal<Voltage> rightSupplyVoltage;
    protected final StatusSignal<Voltage> rightVoltage;
    protected final StatusSignal<Temperature> rightTemperature;

    private final Debouncer leftConnected = new Debouncer(0.5);
    private final Debouncer rightConnected = new Debouncer(0.5);

    public IntakeIOKraken() {
        leftMotor = new TalonFX(LEFT_INTAKE_MOTOR_ID, CAN_BUS);
        rightMotor = new TalonFX(RIGHT_INTAKE_MOTOR_ID, CAN_BUS);

        Slot0Configs slotConfigs = new Slot0Configs()
            .withKP(0.8)
            .withKI(0)
            .withKD(0)
            .withKS(0.4)
            .withKV(0.123 * 1.5)
            .withKA(0);
        
        MotionMagicConfigs magicConfigs = new MotionMagicConfigs();

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Slot0 = slotConfigs;
        motorConfig.Feedback.SensorToMechanismRatio = 1.5;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotionMagic = magicConfigs;

        leftMotor.getConfigurator().apply(motorConfig);
        rightMotor.getConfigurator().apply(motorConfig);
        rightMotor.setControl(new Follower(LEFT_INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));

        leftPosition = leftMotor.getPosition();
        leftStatorCurrent = leftMotor.getStatorCurrent();
        leftSupplyCurrent = leftMotor.getSupplyCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftVelocity = leftMotor.getVelocity();
        leftSupplyVoltage = leftMotor.getSupplyVoltage();
        leftVoltage = leftMotor.getMotorVoltage();
        leftTemperature = leftMotor.getDeviceTemp();

        rightPosition = rightMotor.getPosition();
        rightStatorCurrent = rightMotor.getStatorCurrent();
        rightSupplyCurrent = rightMotor.getSupplyCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightVelocity = rightMotor.getVelocity();
        rightSupplyVoltage = rightMotor.getSupplyVoltage();
        rightVoltage = rightMotor.getMotorVoltage();
        rightTemperature = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            leftPosition,
            leftStatorCurrent,
            leftSupplyCurrent,
            leftTorqueCurrent,
            leftVelocity,
            leftSupplyVoltage,
            leftVoltage,
            leftTemperature,
            rightPosition,
            rightStatorCurrent,
            rightSupplyCurrent,
            rightTorqueCurrent,
            rightVelocity,
            rightSupplyVoltage,
            rightVoltage,
            rightTemperature
            );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode leftStatus = BaseStatusSignal.refreshAll(
            leftPosition,
            leftStatorCurrent,
            leftSupplyCurrent,
            leftTorqueCurrent,
            leftVelocity,
            leftSupplyVoltage,
            leftVoltage,
            leftTemperature
        );

        StatusCode rightStatus = BaseStatusSignal.refreshAll(
            rightPosition,
            rightStatorCurrent,
            rightSupplyCurrent,
            rightTorqueCurrent,
            rightVelocity,
            rightSupplyVoltage,
            rightVoltage,
            rightTemperature
        );

        inputs.leftIsConnected = leftConnected.calculate(leftStatus.isOK());
        inputs.leftPosition = Units.rotationsToRadians(leftMotor.getPosition().getValueAsDouble());
        inputs.leftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.leftSupplyCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.leftTorqueCurrent = leftMotor.getTorqueCurrent().getValueAsDouble();
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftMotor.getVelocity().getValueAsDouble());
        inputs.leftSupplyVolts = leftMotor.getSupplyVoltage().getValueAsDouble();
        inputs.leftMotorVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftTemperature = leftMotor.getDeviceTemp().getValueAsDouble();

        inputs.rightIsConnected = rightConnected.calculate(rightStatus.isOK());
        inputs.rightPosition = Units.rotationsToRadians(rightMotor.getPosition().getValueAsDouble());
        inputs.rightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightSupplyCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rightTorqueCurrent = rightMotor.getTorqueCurrent().getValueAsDouble();
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightMotor.getVelocity().getValueAsDouble());
        inputs.rightSupplyVolts = rightMotor.getSupplyVoltage().getValueAsDouble();
        inputs.rightMotorVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightTemperature = rightMotor.getDeviceTemp().getValueAsDouble();

        inputs.extendDistance = (inputs.leftPosition + inputs.rightPosition) / 2;
    }

    @Override
    public void setIntakePosition(double inches) {
        leftMotor.setControl(positionTorqueCurrentRequest.withPosition(inches));
    }
}
