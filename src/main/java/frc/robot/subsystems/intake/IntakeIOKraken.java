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
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

public class IntakeIOKraken implements IntakeIO {
    protected static final double inPerRot = 0.5;

    protected final TalonFX leftMotor;
    protected final TalonFX rightMotor;
    protected final TalonFX roller;

    //control requests
    protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);

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

    protected final StatusSignal<Angle> rollerPosition;
    protected final StatusSignal<Current> rollerStatorCurrent;
    protected final StatusSignal<Current> rollerSupplyCurrent;
    protected final StatusSignal<Current> rollerTorqueCurrent;
    protected final StatusSignal<AngularVelocity> rollerVelocity;
    protected final StatusSignal<Voltage> rollerSupplyVoltage;
    protected final StatusSignal<Voltage> rollerVoltage;
    protected final StatusSignal<Temperature> rollerTemperature;

    private final Debouncer leftConnected = new Debouncer(0.5);
    private final Debouncer rightConnected = new Debouncer(0.5);
    private final Debouncer rollerConnected = new Debouncer(0.5);

    public IntakeIOKraken() {
        leftMotor = new TalonFX(20, CAN_BUS);
        rightMotor = new TalonFX(21, CAN_BUS);
        roller = new TalonFX(22, CAN_BUS);

        Slot0Configs extensionSlotConfigs = new Slot0Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0)
            .withKS(0.4)
            .withKV(0.123 * 46.0 / 11.0)
            .withKA(0);
        Slot0Configs rollerSlotConfigs = new Slot0Configs()
            .withKP(15)
            .withKI(0)
            .withKD(0)
            .withKS(0.4)
            .withKV(0.123 * 24.0 / 11.0)
            .withKA(0);
        
        MotionMagicConfigs magicConfigs = new MotionMagicConfigs();

        TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();
            extensionMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
            extensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            extensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
            extensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            extensionMotorConfig.Slot0 = extensionSlotConfigs;
            extensionMotorConfig.Feedback.SensorToMechanismRatio = 46.0 / 11.0;
            extensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            extensionMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 60.0;
            extensionMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            extensionMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -10.0;
            extensionMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            extensionMotorConfig.MotionMagic = magicConfigs;
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
            rollerConfig.CurrentLimits.StatorCurrentLimit = 120;
            rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            rollerConfig.CurrentLimits.SupplyCurrentLimit = 70;
            rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
            rollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.75;
            rollerConfig.Slot0 = rollerSlotConfigs;
            rollerConfig.Feedback.SensorToMechanismRatio = 24.0 / 11.0;
            rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftMotor.getConfigurator().apply(extensionMotorConfig);
        rightMotor.getConfigurator().apply(extensionMotorConfig);
        rightMotor.setControl(new Follower(20, MotorAlignmentValue.Opposed));
        roller.getConfigurator().apply(rollerConfig);

        leftMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);

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

        rollerPosition = roller.getPosition();
        rollerStatorCurrent = roller.getStatorCurrent();
        rollerSupplyCurrent = roller.getSupplyCurrent();
        rollerTorqueCurrent = roller.getTorqueCurrent();
        rollerVelocity = roller.getVelocity();
        rollerSupplyVoltage = roller.getSupplyVoltage();
        rollerVoltage = roller.getMotorVoltage();
        rollerTemperature = roller.getDeviceTemp();

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
            rightTemperature,
            rollerPosition,
            rollerStatorCurrent,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerVelocity,
            rollerSupplyVoltage,
            rollerVoltage,
            rollerTemperature
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

        StatusCode rollerStatus = BaseStatusSignal.refreshAll(
            rollerPosition,
            rollerStatorCurrent,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerVelocity,
            rollerSupplyVoltage,
            rollerVoltage,
            rollerTemperature
        );

        inputs.leftIsConnected = leftConnected.calculate(leftStatus.isOK());
        inputs.leftPosition = leftMotor.getPosition().getValueAsDouble();
        inputs.leftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.leftSupplyCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.leftTorqueCurrent = leftMotor.getTorqueCurrent().getValueAsDouble();
        inputs.leftVelocityRotPerSec = leftMotor.getVelocity().getValueAsDouble();
        inputs.leftSupplyVolts = leftMotor.getSupplyVoltage().getValueAsDouble();
        inputs.leftMotorVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftTemperature = leftMotor.getDeviceTemp().getValueAsDouble();

        inputs.rightIsConnected = rightConnected.calculate(rightStatus.isOK());
        inputs.rightPosition = rightMotor.getPosition().getValueAsDouble();
        inputs.rightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightSupplyCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rightTorqueCurrent = rightMotor.getTorqueCurrent().getValueAsDouble();
        inputs.rightVelocityRotPerSec = rightMotor.getVelocity().getValueAsDouble();
        inputs.rightSupplyVolts = rightMotor.getSupplyVoltage().getValueAsDouble();
        inputs.rightMotorVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightTemperature = rightMotor.getDeviceTemp().getValueAsDouble();

        inputs.rollerIsConnected = rollerConnected.calculate(rollerStatus.isOK());
        inputs.rollerPosition = roller.getPosition().getValueAsDouble();
        inputs.rollerStatorCurrent = roller.getStatorCurrent().getValueAsDouble();
        inputs.rollerSupplyCurrent = roller.getSupplyCurrent().getValueAsDouble();
        inputs.rollerTorqueCurrent = roller.getTorqueCurrent().getValueAsDouble();
        inputs.rollerVelocityRotPerSec = roller.getVelocity().getValueAsDouble();
        inputs.rollerSupplyVolts = roller.getSupplyVoltage().getValueAsDouble();
        inputs.rollerMotorVolts = roller.getMotorVoltage().getValueAsDouble();
        inputs.rollerTemperature = roller.getDeviceTemp().getValueAsDouble();

        inputs.extendDistance = (inputs.leftPosition + inputs.rightPosition) / 2 * inPerRot;
        inputs.isRunning = (inputs.extendDistance > 11 ? true : false) && inputs.rollerVelocityRotPerSec > 40;
    }

    @Override
    public void setIntakeState(double inches, double rotPerSec) {
        setIntakeDistance(inches);
        setRollerSpeed(rotPerSec);
    }

    @Override
    public void deployIntake() {
        setIntakeState(12, 50);;
    }

    @Override
    public void stopIntake() {
        leftMotor.setControl(voltageRequest);
        roller.setControl(voltageRequest);
    }
    @Override
    public void setIntakeDistance(double inches) {
        leftMotor.setControl(positionTorqueCurrentRequest.withPosition(inches / inPerRot));
    }

    @Override
    public void setRollerSpeed(double rotPerSec) {
        roller.setControl(velocityTorqueRequest.withVelocity(rotPerSec));
    }
}
