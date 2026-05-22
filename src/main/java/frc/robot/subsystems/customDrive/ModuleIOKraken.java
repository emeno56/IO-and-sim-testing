package frc.robot.subsystems.customDrive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
 

public class ModuleIOKraken implements ModuleIO{
    //specific to the module constants
    protected final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    //hardware
    protected final TalonFX drive;
    protected final TalonFX steer;
    protected final CANcoder absoluteEncoder;

    // Openloop control requests
    protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0).withEnableFOC(true);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0).withEnableFOC(true);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Motion Magic for steer
    protected final MotionMagicVoltage steerRequest = new MotionMagicVoltage(0);

    //drive inputs
    protected final StatusSignal<Angle> drivePosition;
    protected final StatusSignal<Current> driveStatorCurrent;
    protected final StatusSignal<Current> driveSupplyCurrent;
    protected final StatusSignal<Current> driveTorqueCurrent;
    protected final StatusSignal<AngularVelocity> driveVelocity;
    protected final StatusSignal<Voltage> driveSupplyVoltage;
    protected final StatusSignal<Voltage> driveMotorVoltage;
    protected final StatusSignal<Temperature> driveTemperature;

    //steer inputs
    protected final StatusSignal<Angle> steerPosition;
    protected final StatusSignal<Angle> steerAbsolutePosition;
    protected final StatusSignal<Current> steerStatorCurrent;
    protected final StatusSignal<Current> steerSupplyCurrent;
    protected final StatusSignal<Current> steerTorqueCurrent;
    protected final StatusSignal<AngularVelocity> steerVelocity;
    protected final StatusSignal<Voltage> steerSupplyVoltage;
    protected final StatusSignal<Voltage> steerMotorVoltage;
    protected final StatusSignal<Temperature> steerTemperature; //position

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer absoluteEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOKraken(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        //create hardware
        drive = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
        steer = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
        absoluteEncoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

        //configure drive
        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
            driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
            driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            driveConfig.Slot0 = constants.DriveMotorGains;
            driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
            driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
            driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
            driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            tryUntilOk(5, () -> drive.getConfigurator().apply(driveConfig, 0.25));
            tryUntilOk(5, () -> drive.setPosition(0));

        //configure steer
        TalonFXConfiguration steerConfig = constants.SteerMotorInitialConfigs;
            steerConfig.MotorOutput.Inverted = constants.SteerMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            steerConfig.Slot0 = constants.SteerMotorGains;
            steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
            steerConfig.Feedback.FeedbackSensorSource = switch(constants.FeedbackSource) {
                case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
                case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
                case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
                default -> throw new RuntimeException("Unsupported Swerve Configuration and ModuleIOKraken.java, line 107");
            };
            steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
            steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
            steerConfig.MotionMagic.MotionMagicAcceleration = 
            steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
            steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
            steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
            steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
            tryUntilOk(5, () -> steer.getConfigurator().apply(steerConfig, 0.25));

        //configure CANCoder
        CANcoderConfiguration encoderConfig = constants.EncoderInitialConfigs;
            encoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
            encoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
            tryUntilOk(5, () -> absoluteEncoder.getConfigurator().apply(encoderConfig, 0.25));
        
        //create status signals
            

            //Drive signals
            drivePosition = drive.getPosition();
            driveStatorCurrent = drive.getStatorCurrent();
            driveSupplyCurrent = drive.getSupplyCurrent();
            driveTorqueCurrent = drive.getTorqueCurrent();
            driveVelocity = drive.getVelocity();
            driveSupplyVoltage = drive.getSupplyVoltage();
            driveMotorVoltage = drive.getMotorVoltage();
            driveTemperature = drive.getDeviceTemp();

            //Steer signals
            steerPosition = steer.getPosition();
            steerAbsolutePosition = absoluteEncoder.getAbsolutePosition();
            steerStatorCurrent = steer.getStatorCurrent();
            steerSupplyCurrent = steer.getSupplyCurrent();
            steerTorqueCurrent = steer.getTorqueCurrent();
            steerVelocity = steer.getVelocity();
            steerSupplyVoltage = steer.getMotorVoltage();
            steerMotorVoltage = steer.getMotorVoltage();
            steerTemperature = steer.getDeviceTemp();

        //configure refresh
        BaseStatusSignal.setUpdateFrequencyForAll(
            Drive.ODOMETRY_FREQUENCY, 
            drivePosition,
            steerPosition,
            steerAbsolutePosition,
            driveVelocity,
            steerVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            driveStatorCurrent,
            driveSupplyCurrent,
            driveTorqueCurrent,
            driveSupplyVoltage,
            driveMotorVoltage,
            driveTemperature,
            steerStatorCurrent,
            steerSupplyCurrent,
            steerTorqueCurrent,
            steerSupplyVoltage,
            steerMotorVoltage,
            steerTemperature
        );
        
        drive.optimizeBusUtilization();
        steer.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        //grab the motor status codes
        StatusCode driveStatus = BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveStatorCurrent,
            driveSupplyCurrent,
            driveTorqueCurrent,
            driveSupplyVoltage,
            driveMotorVoltage,
            driveTemperature
        );
        StatusCode steerStatus = BaseStatusSignal.refreshAll(
            steerPosition,
            steerVelocity,
            steerStatorCurrent,
            steerSupplyCurrent,
            steerTorqueCurrent,
            steerSupplyVoltage,
            steerMotorVoltage,
            steerTemperature
        );
        StatusCode absoluteStatus = BaseStatusSignal.refreshAll(steerAbsolutePosition);

        //update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePosition = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveStatorCurrent = driveStatorCurrent.getValueAsDouble();
        inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
        inputs.driveTorqueCurrent = driveTorqueCurrent.getValueAsDouble();
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveSupplyVolts = driveSupplyVoltage.getValueAsDouble();
        inputs.driveMotorVolts = driveMotorVoltage.getValueAsDouble();
        inputs.driveTemperature = driveTemperature.getValueAsDouble();

        //update steer inputs
        inputs.steerConnected = steerConnectedDebounce.calculate(steerStatus.isOK());
        inputs.absoluteEncoderConnected = absoluteEncoderConnectedDebounce.calculate(absoluteStatus.isOK());
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
        inputs.absolutePosition = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
        inputs.steerStatorCurrent = steerStatorCurrent.getValueAsDouble();
        inputs.steerSupplyCurrent = steerSupplyCurrent.getValueAsDouble();
        inputs.steerTorqueCurrent = steerTorqueCurrent.getValueAsDouble();
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
        inputs.steerSupplyVolts = steerSupplyVoltage.getValueAsDouble();
        inputs.steerMotorVolts = steerMotorVoltage.getValueAsDouble();
        inputs.steerTemperature = steerTemperature.getValueAsDouble();
    }

    //define helpful interface methods
    @Override
    public void setDriveOpenLoop(double output) {
        drive.setControl(
            switch(constants.DriveMotorClosedLoopOutput) {
                case Voltage -> voltageRequest.withOutput(output).withEnableFOC(true);
                case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            }
        );
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steer.setControl(
            switch(constants.SteerMotorClosedLoopOutput) {
                case Voltage -> voltageRequest.withOutput(output).withEnableFOC(true);
                case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            }
        );
    }

    @Override
    public void setDriveVelocity(double driveVelocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(driveVelocityRadPerSec) * TunerConstants.FrontLeft.DriveMotorGearRatio;
        drive.setControl(
            switch(constants.DriveMotorClosedLoopOutput) {
                case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec).withEnableFOC(true);
                case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
            }
        );
    }

    @Override
    public void setSteerPosition(Rotation2d pos) {
        steer.setControl(
          switch(constants.SteerMotorClosedLoopOutput) {
            case Voltage -> positionVoltageRequest.withPosition(pos.getRotations());
            case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(pos.getRotations());
          }  
        );
    }
}
