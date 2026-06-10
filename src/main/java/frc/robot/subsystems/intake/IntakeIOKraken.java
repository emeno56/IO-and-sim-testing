package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOKraken implements IntakeIO {
    public static final double EXTENSION_GEARING = 4.181818;
    public static final double ROLLER_GEARING = 2.181818;
    protected static final double inPerRot = 0.5;

    private static final int leftID = 21;
    private static final int rightID = 22;
    private static final int rollerID = 20;

    protected final TalonFX leftMotor;
    protected final TalonFX rightMotor;
    protected final TalonFX roller;

    //control requests
    protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    protected final MotionMagicVoltage mmvRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0).withEnableFOC(true);
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
        leftMotor = new TalonFX(leftID, CAN_BUS);
        rightMotor = new TalonFX(rightID, CAN_BUS);
        roller = new TalonFX(rollerID, CAN_BUS);

        Slot0Configs extensionSlotConfigs = new Slot0Configs()
            .withKP(2.5)
            .withKI(0)
            .withKD(2)
            .withKS(0)
            .withKV(2)
            .withKA(.1);
        Slot1Configs deploySlotConfigs = new Slot1Configs()
            .withKP(10.0)
            .withKI(0)
            .withKD(1.5)
            .withKS(0)
            .withKV(2.0)
            .withKA(0);

        Slot0Configs rollerSlotConfigs = new Slot0Configs()
            .withKP(15)
            .withKI(0)
            .withKD(0)
            .withKS(0.4)
            .withKV(0.12413 * ROLLER_GEARING)
            .withKA(0);
        
        MotionMagicConfigs magicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(4.5)
            .withMotionMagicAcceleration(28)
            .withMotionMagicJerk(0);

        TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();
            extensionMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
            extensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            extensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
            extensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            extensionMotorConfig.Slot0 = extensionSlotConfigs;
            extensionMotorConfig.Slot1 = deploySlotConfigs;
            extensionMotorConfig.Feedback.SensorToMechanismRatio = EXTENSION_GEARING;
            extensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            extensionMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.5 / inPerRot;
            extensionMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            extensionMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
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
            rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_GEARING;
            rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftMotor.getConfigurator().apply(extensionMotorConfig);
        rightMotor.getConfigurator().apply(extensionMotorConfig);
        rightMotor.setControl(new Follower(leftID, MotorAlignmentValue.Opposed));
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
        inputs.leftPosition = leftPosition.getValueAsDouble();
        inputs.leftStatorCurrent = leftStatorCurrent.getValueAsDouble();
        inputs.leftSupplyCurrent = leftSupplyCurrent.getValueAsDouble();
        inputs.leftTorqueCurrent = leftTorqueCurrent.getValueAsDouble();
        inputs.leftVelocityRotPerSec = leftVelocity.getValueAsDouble();
        inputs.leftSupplyVolts = leftSupplyVoltage.getValueAsDouble();
        inputs.leftMotorVolts = leftVoltage.getValueAsDouble();
        inputs.leftTemperature = leftTemperature.getValueAsDouble();

        inputs.rightIsConnected = rightConnected.calculate(rightStatus.isOK());
        inputs.rightPosition = rightPosition.getValueAsDouble();
        inputs.rightStatorCurrent = rightStatorCurrent.getValueAsDouble();
        inputs.rightSupplyCurrent = rightSupplyCurrent.getValueAsDouble();
        inputs.rightTorqueCurrent = rightTorqueCurrent.getValueAsDouble();
        inputs.rightVelocityRotPerSec = rightVelocity.getValueAsDouble();
        inputs.rightSupplyVolts = rightSupplyVoltage.getValueAsDouble();
        inputs.rightMotorVolts = rightVoltage.getValueAsDouble();
        inputs.rightTemperature = rightTemperature.getValueAsDouble();

        inputs.rollerIsConnected = rollerConnected.calculate(rollerStatus.isOK());
        inputs.rollerPosition = rollerPosition.getValueAsDouble();
        inputs.rollerStatorCurrent = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerSupplyCurrent = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerTorqueCurrent = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerVelocityRotPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerSupplyVolts = rollerSupplyVoltage.getValueAsDouble();
        inputs.rollerMotorVolts = rollerVoltage.getValueAsDouble();
        inputs.rollerTemperature = rollerTemperature.getValueAsDouble();

        inputs.extendDistance = (inputs.leftPosition + inputs.rightPosition) / 2 * inPerRot;
        inputs.isRunning = (inputs.extendDistance > 10 ? true : false) && inputs.rollerVelocityRotPerSec > 25;
    }

    @Override
    public void setIntakeState(double inches, double rotPerSec) {
        Logger.recordOutput("Intake/Desired/distance", inches);
        Logger.recordOutput("Intake/Desired/rotPerSec", rotPerSec);
        setIntakeDistance(inches);
        setRollerSpeed(rotPerSec);
    }

    @Override
    public void deployIntake() {
        Logger.recordOutput("Intake/Desired/distance", 10.5);
        Logger.recordOutput("Intake/Desired/rotPerSec", 50);
        leftMotor.setControl(positionVoltageRequest.withPosition(10.5 / inPerRot).withSlot(1));
        roller.setControl(velocityTorqueRequest.withVelocity(50));
    }

    @Override
    public void stopIntake() {
        leftMotor.setControl(voltageRequest);
        roller.setControl(voltageRequest);
    }
    @Override
    public void setIntakeDistance(double inches) {
        leftMotor.setControl(mmvRequest.withPosition(inches / inPerRot));
    }

    @Override
    public void setRollerSpeed(double rotPerSec) {
        roller.setControl(velocityTorqueRequest.withVelocity(rotPerSec));
    }
}
