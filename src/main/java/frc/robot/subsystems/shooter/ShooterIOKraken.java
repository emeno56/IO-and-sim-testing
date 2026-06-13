package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOKraken implements ShooterIO {
    public static final double DRUM_GEARING = 0.25;
    public static final double HOOD_GEARING = 47.454545;

    private final int hoodID = 40;
    private final int m1ID = 41;
    private final int m2ID = 42;
    private final int m3ID = 43;
    private final int m4ID = 44;

    protected final TalonFX hood;
    protected final TalonFX motor1;
    protected final TalonFX motor2;
    protected final TalonFX motor3;
    protected final TalonFX motor4;

    protected final TalonFX[] drums = new TalonFX[4];

    protected final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
    protected final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    protected final VoltageOut vOutRequest = new VoltageOut(0);
    
    protected final StatusSignal<Angle> motor1Position;
    protected final StatusSignal<Current> motor1StatorCurrent;
    protected final StatusSignal<Current> motor1SupplyCurrent;
    protected final StatusSignal<Current> motor1TorqueCurrent;
    protected final StatusSignal<AngularVelocity> motor1Velocity;
    protected final StatusSignal<Voltage> motor1SupplyVoltage;
    protected final StatusSignal<Voltage> motor1Voltage;
    protected final StatusSignal<Temperature> motor1Temperature;

    protected final StatusSignal<Angle> motor2Position;
    protected final StatusSignal<Current> motor2StatorCurrent;
    protected final StatusSignal<Current> motor2SupplyCurrent;
    protected final StatusSignal<Current> motor2TorqueCurrent;
    protected final StatusSignal<AngularVelocity> motor2Velocity;
    protected final StatusSignal<Voltage> motor2SupplyVoltage;
    protected final StatusSignal<Voltage> motor2Voltage;
    protected final StatusSignal<Temperature> motor2Temperature;

    protected final StatusSignal<Angle> motor3Position;
    protected final StatusSignal<Current> motor3StatorCurrent;
    protected final StatusSignal<Current> motor3SupplyCurrent;
    protected final StatusSignal<Current> motor3TorqueCurrent;
    protected final StatusSignal<AngularVelocity> motor3Velocity;
    protected final StatusSignal<Voltage> motor3SupplyVoltage;
    protected final StatusSignal<Voltage> motor3Voltage;
    protected final StatusSignal<Temperature> motor3Temperature;

    protected final StatusSignal<Angle> motor4Position;
    protected final StatusSignal<Current> motor4StatorCurrent;
    protected final StatusSignal<Current> motor4SupplyCurrent;
    protected final StatusSignal<Current> motor4TorqueCurrent;
    protected final StatusSignal<AngularVelocity> motor4Velocity;
    protected final StatusSignal<Voltage> motor4SupplyVoltage;
    protected final StatusSignal<Voltage> motor4Voltage;
    protected final StatusSignal<Temperature> motor4Temperature;

    protected final StatusSignal<Angle> hoodPosition;
    protected final StatusSignal<Current> hoodStatorCurrent;
    protected final StatusSignal<Current> hoodSupplyCurrent;
    protected final StatusSignal<Current> hoodTorqueCurrent;
    protected final StatusSignal<AngularVelocity> hoodVelocity;
    protected final StatusSignal<Voltage> hoodSupplyVoltage;
    protected final StatusSignal<Voltage> hoodVoltage;
    protected final StatusSignal<Temperature> hoodTemperature;

    private final Debouncer motor1ConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer motor2ConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer motor3ConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer motor4ConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5);

    public ShooterIOKraken()  {
        motor1 = new TalonFX(m1ID);
        motor2 = new TalonFX(m2ID);
        motor3 = new TalonFX(m3ID);
        motor4 = new TalonFX(m4ID);
        hood = new TalonFX(hoodID);

        Slot0Configs shooterGains = new Slot0Configs();
            shooterGains.kP = 7.5;
            shooterGains.kI = 0.0;
            shooterGains.kD = 0.0;
            shooterGains.kS = 0.0;
            shooterGains.kG = 0.0;
            shooterGains.kV = 0.12413 * DRUM_GEARING;
            shooterGains.kA = 0.0;
        Slot0Configs hoodGains = new Slot0Configs();
            hoodGains.kP = 70.0;
            hoodGains.kI = 0.0;
            hoodGains.kD = 0.0;
            hoodGains.kS = 0.42;
            hoodGains.kG = 0.0;
            hoodGains.kV = 0.0;
            hoodGains.kA = 0.0;

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
            shooterConfig.CurrentLimits.StatorCurrentLimit = 120;
            shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            shooterConfig.CurrentLimits.SupplyCurrentLimit = 80;
            shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            shooterConfig.Slot0 = shooterGains;
            shooterConfig.Feedback.SensorToMechanismRatio = DRUM_GEARING;
            shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
            hoodConfig.CurrentLimits.StatorCurrentLimit = 80;
            hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            hoodConfig.CurrentLimits.SupplyCurrentLimit = 60;
            hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            hoodConfig.Slot0 = hoodGains;
            hoodConfig.Feedback.SensorToMechanismRatio = HOOD_GEARING;
            hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            hoodConfig.ClosedLoopGeneral.ContinuousWrap = true;
            // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable

        motor1.getConfigurator().apply(shooterConfig);
        motor2.getConfigurator().apply(shooterConfig);
        motor3.getConfigurator().apply(shooterConfig);
        motor4.getConfigurator().apply(shooterConfig);
        
        motor2.setControl(new Follower(m1ID, MotorAlignmentValue.Aligned));
        motor3.setControl(new Follower(m1ID, MotorAlignmentValue.Aligned));
        motor4.setControl(new Follower(m1ID, MotorAlignmentValue.Aligned));
        
        hood.getConfigurator().apply(hoodConfig);
        hood.setPosition(0);

        motor1Position = motor1.getPosition();
        motor1StatorCurrent = motor1.getStatorCurrent();
        motor1SupplyCurrent = motor1.getSupplyCurrent();
        motor1TorqueCurrent = motor1.getTorqueCurrent();
        motor1Velocity = motor1.getVelocity();
        motor1SupplyVoltage = motor1.getSupplyVoltage();
        motor1Voltage = motor1.getMotorVoltage();
        motor1Temperature = motor1.getDeviceTemp();

        motor2Position = motor2.getPosition();
        motor2StatorCurrent = motor2.getStatorCurrent();
        motor2SupplyCurrent = motor2.getSupplyCurrent();
        motor2TorqueCurrent = motor2.getTorqueCurrent();
        motor2Velocity = motor2.getVelocity();
        motor2SupplyVoltage = motor2.getSupplyVoltage();
        motor2Voltage = motor2.getMotorVoltage();
        motor2Temperature = motor2.getDeviceTemp();

        motor3Position = motor3.getPosition();
        motor3StatorCurrent = motor3.getStatorCurrent();
        motor3SupplyCurrent = motor3.getSupplyCurrent();
        motor3TorqueCurrent = motor3.getTorqueCurrent();
        motor3Velocity = motor3.getVelocity();
        motor3SupplyVoltage = motor3.getSupplyVoltage();
        motor3Voltage = motor3.getMotorVoltage();
        motor3Temperature = motor3.getDeviceTemp();

        motor4Position = motor4.getPosition();
        motor4StatorCurrent = motor4.getStatorCurrent();
        motor4SupplyCurrent = motor4.getSupplyCurrent();
        motor4TorqueCurrent = motor4.getTorqueCurrent();
        motor4Velocity = motor4.getVelocity();
        motor4SupplyVoltage = motor4.getSupplyVoltage();
        motor4Voltage = motor4.getMotorVoltage();
        motor4Temperature = motor4.getDeviceTemp();

        hoodPosition = hood.getPosition();
        hoodStatorCurrent = hood.getStatorCurrent();
        hoodSupplyCurrent = hood.getSupplyCurrent();
        hoodTorqueCurrent = hood.getTorqueCurrent();
        hoodVelocity = hood.getVelocity();
        hoodSupplyVoltage = hood.getSupplyVoltage();
        hoodVoltage = hood.getMotorVoltage();
        hoodTemperature = hood.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, 
            motor1Position,
            motor1StatorCurrent,
            motor1SupplyCurrent,
            motor1TorqueCurrent,
            motor1Velocity,
            motor1SupplyVoltage,
            motor1Voltage,
            motor1Temperature,
            motor2Position,
            motor2StatorCurrent,
            motor2SupplyCurrent,
            motor2TorqueCurrent,
            motor2Velocity,
            motor2SupplyVoltage,
            motor2Voltage,
            motor2Temperature,
            motor3Position,
            motor3StatorCurrent,
            motor3SupplyCurrent,
            motor3TorqueCurrent,
            motor3Velocity,
            motor3SupplyVoltage,
            motor3Voltage,
            motor3Temperature,
            motor4Position,
            motor4StatorCurrent,
            motor4SupplyCurrent,
            motor4TorqueCurrent,
            motor4Velocity,
            motor4SupplyVoltage,
            motor4Voltage,
            motor4Temperature,
            hoodPosition,
            hoodStatorCurrent,
            hoodSupplyCurrent,
            hoodTorqueCurrent,
            hoodVelocity,
            hoodSupplyVoltage,
            hoodVoltage,
            hoodTemperature
        );

        drums[0] = motor1;
        drums[1] = motor2;
        drums[2] = motor3;
        drums[3] = motor4;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusCode motor1Status = BaseStatusSignal.refreshAll(
            motor1Position,
            motor1StatorCurrent,
            motor1SupplyCurrent,
            motor1TorqueCurrent,
            motor1Velocity,
            motor1SupplyVoltage,
            motor1Voltage,
            motor1Temperature
        );

        StatusCode motor2Status = BaseStatusSignal.refreshAll(
            motor2Position,
            motor2StatorCurrent,
            motor2SupplyCurrent,
            motor2TorqueCurrent,
            motor2Velocity,
            motor2SupplyVoltage,
            motor2Voltage,
            motor2Temperature
        );

        StatusCode motor3Status = BaseStatusSignal.refreshAll(
            motor3Position,
            motor3StatorCurrent,
            motor3SupplyCurrent,
            motor3TorqueCurrent,
            motor3Velocity,
            motor3SupplyVoltage,
            motor3Voltage,
            motor3Temperature
        );

        StatusCode motor4Status = BaseStatusSignal.refreshAll(
            motor4Position,
            motor4StatorCurrent,
            motor4SupplyCurrent,
            motor4TorqueCurrent,
            motor4Velocity,
            motor4SupplyVoltage,
            motor4Voltage,
            motor4Temperature
        );

        StatusCode hoodStatus = BaseStatusSignal.refreshAll(
            hoodPosition,
            hoodStatorCurrent,
            hoodSupplyCurrent,
            hoodTorqueCurrent,
            hoodVelocity,
            hoodSupplyVoltage,
            hoodVoltage,
            hoodTemperature
        );

        inputs.motor1IsConnected = motor1ConnectedDebouncer.calculate(motor1Status.isOK());
        inputs.motor1Position = motor1Position.getValueAsDouble();
        inputs.motor1StatorCurrent = motor1StatorCurrent.getValueAsDouble();
        inputs.motor1SupplyCurrent = motor1SupplyCurrent.getValueAsDouble();
        inputs.motor1TorqueCurrent = motor1TorqueCurrent.getValueAsDouble();
        inputs.motor1VelocityRotPerSec = motor1Velocity.getValueAsDouble();
        inputs.motor1SupplyVolts = motor1SupplyVoltage.getValueAsDouble();
        inputs.motor1MotorVolts = motor1Voltage.getValueAsDouble();
        inputs.motor1Temperature = motor1Temperature.getValueAsDouble();

        inputs.motor2IsConnected = motor2ConnectedDebouncer.calculate(motor2Status.isOK());
        inputs.motor2Position = motor2Position.getValueAsDouble();
        inputs.motor2StatorCurrent = motor2StatorCurrent.getValueAsDouble();
        inputs.motor2SupplyCurrent = motor2SupplyCurrent.getValueAsDouble();
        inputs.motor2TorqueCurrent = motor2TorqueCurrent.getValueAsDouble();
        inputs.motor2VelocityRotPerSec = motor2Velocity.getValueAsDouble();
        inputs.motor2SupplyVolts = motor2SupplyVoltage.getValueAsDouble();
        inputs.motor2MotorVolts = motor2Voltage.getValueAsDouble();
        inputs.motor2Temperature = motor2Temperature.getValueAsDouble();

        inputs.motor3IsConnected = motor3ConnectedDebouncer.calculate(motor3Status.isOK());
        inputs.motor3Position = motor3Position.getValueAsDouble();
        inputs.motor3StatorCurrent = motor3StatorCurrent.getValueAsDouble();
        inputs.motor3SupplyCurrent = motor3SupplyCurrent.getValueAsDouble();
        inputs.motor3TorqueCurrent = motor3TorqueCurrent.getValueAsDouble();
        inputs.motor3VelocityRotPerSec = motor3Velocity.getValueAsDouble();
        inputs.motor3SupplyVolts = motor3SupplyVoltage.getValueAsDouble();
        inputs.motor3MotorVolts = motor3Voltage.getValueAsDouble();
        inputs.motor3Temperature = motor3Temperature.getValueAsDouble();

        inputs.motor4IsConnected = motor4ConnectedDebouncer.calculate(motor4Status.isOK());
        inputs.motor4Position = motor4Position.getValueAsDouble();
        inputs.motor4StatorCurrent = motor4StatorCurrent.getValueAsDouble();
        inputs.motor4SupplyCurrent = motor4SupplyCurrent.getValueAsDouble();
        inputs.motor4TorqueCurrent = motor4TorqueCurrent.getValueAsDouble();
        inputs.motor4VelocityRotPerSec = motor4Velocity.getValueAsDouble();
        inputs.motor4SupplyVolts = motor4SupplyVoltage.getValueAsDouble();
        inputs.motor4MotorVolts = motor4Voltage.getValueAsDouble();
        inputs.motor4Temperature = motor4Temperature.getValueAsDouble();

        inputs.hoodIsConnected = hoodConnectedDebouncer.calculate(hoodStatus.isOK());
        inputs.hoodPosition = Units.rotationsToDegrees(hoodPosition.getValueAsDouble());
        inputs.hoodStatorCurrent = hoodStatorCurrent.getValueAsDouble();
        inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
        inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
        inputs.hoodVelocityRotPerSec = hoodVelocity.getValueAsDouble();
        inputs.hoodSupplyVolts = hoodSupplyVoltage.getValueAsDouble();
        inputs.hoodMotorVolts = hoodVoltage.getValueAsDouble();
        inputs.hoodTemperature = hoodTemperature.getValueAsDouble();

        inputs.fuelExitVelocityMPS = rpsToMps((inputs.motor1VelocityRotPerSec + inputs.motor2VelocityRotPerSec + inputs.motor3VelocityRotPerSec + inputs.motor4VelocityRotPerSec) / 4.0); //average speed from all motors
    }

    @Override
    public void setShooterState(double rps, double degrees) {
        setHoodAngle(degrees);
        setFuelExitVelocity(rps);
    }

    @Override
    public void setHoodAngle(double degrees) {
        Logger.recordOutput("Shooter/Desired/Degrees", degrees);
        double rotations = Units.degreesToRotations(degrees);
        hood.setControl(positionRequest.withPosition(rotations));
    }

    @Override
    public void setFuelExitVelocity(double mps) {
        Logger.recordOutput("Shooter/Desired/MPS", mps);
        motor1.setControl(velocityRequest.withVelocity(mpsToRps(mps)));
    }

    @Override
    public void stop() {
        motor1.setControl(vOutRequest);
        setHoodAngle(0);
    }

    public double mpsToRps(double mps) {
        double drumSurfaceSpeed = mps / 1.08; //divide by a compression factor of 1.08
        double drumOmegaRadPerSec = drumSurfaceSpeed / Units.inchesToMeters(3); //divide by drum radius of 2.5 inches
        double motorOmegaRadPerSec = drumOmegaRadPerSec * DRUM_GEARING; //multiply by gear ratio of 1/4
        return motorOmegaRadPerSec / (2 * Math.PI);
    }

    public double rpsToMps(double rps) {
        double motorOmegaRadPerSec = rps * (2 * Math.PI);
        double drumOmegaRadPerSec = motorOmegaRadPerSec / DRUM_GEARING;
        double drumSurfaceSpeed = drumOmegaRadPerSec * Units.inchesToMeters(3);
        return drumSurfaceSpeed * 1.08;
    }
}
