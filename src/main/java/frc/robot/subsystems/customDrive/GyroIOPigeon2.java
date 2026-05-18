package frc.robot.subsystems.customDrive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

//TODO: Merge this file with GyroIOPigeon2. this kind of file making should only be for weird mechanisms like a swerve module
//sim and Real are different enough, normally, that you don't need an inbetwwen file from the source IO and this io.
public class GyroIOPigeon2 implements GyroIO{
    //hardware
    protected final Pigeon2 gyro = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

    //status signals
    protected final StatusSignal<Angle> roll = gyro.getRoll();
    protected final StatusSignal<Angle> pitch = gyro.getPitch();
    protected final StatusSignal<Angle> yaw = gyro.getYaw();
    protected final StatusSignal<AngularVelocity> rollVelocity = gyro.getAngularVelocityXWorld();
    protected final StatusSignal<AngularVelocity> pitchVelocity = gyro.getAngularVelocityXWorld();
    protected final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZWorld();

    // connection debouncer
    private final Debouncer gyroDisconnetDebouncer = new Debouncer(0.5);

    public GyroIOPigeon2() {
        //roll/pitch/yaw mounting config
        MountPoseConfigs config = new MountPoseConfigs();
        config.MountPoseRoll = 0;
        config.MountPosePitch = 0;
        config.MountPoseYaw = 0;
        tryUntilOk(5, () -> gyro.getConfigurator().apply(config, 0.25));
        gyro.optimizeBusUtilization();

        //set statusSignal update Hz
            //faster for odometry
            roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
            pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
            yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
            //slower for data
            rollVelocity.setUpdateFrequency(50.0);
            pitchVelocity.setUpdateFrequency(50.0);
            yawVelocity.setUpdateFrequency(50.0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        StatusCode gyroStatus = BaseStatusSignal.refreshAll(
            roll, pitch, yaw,
            rollVelocity, pitchVelocity, yawVelocity   
        );

        inputs.connected = gyroDisconnetDebouncer.calculate(gyroStatus.isOK());
        inputs.roll = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.pitch = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.fullRotation = new Rotation3d(inputs.roll.getMeasure(), inputs.pitch.getMeasure(), inputs.yaw.getMeasure());
        inputs.rollVelocity = Units.degreesToRadians(rollVelocity.getValueAsDouble());
        inputs.pitchVelocity = Units.degreesToRadians(pitchVelocity.getValueAsDouble());
        inputs.yawVelocity = Units.degreesToRadians(yawVelocity.getValueAsDouble());        
    }
}
