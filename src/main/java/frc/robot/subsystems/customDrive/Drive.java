package frc.robot.subsystems.customDrive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.isRedSide;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;

public class Drive extends SubsystemBase implements Vision.VisionConsumer{
    //Odometry frequency on CANFD is 250 Hz
    public static final double ODOMETRY_FREQUENCY = 250.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
        Math.max(
            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
        Math.max(
            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
    public static final double DRIVE_BASE_HYPOTONUSE = 
        new Translation2d(
            TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY)
        .getDistance(new Translation2d(
            TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));
    public static final double DRIVE_BASE_HALF_WIDTH = Math.abs(TunerConstants.FrontLeft.LocationY);
    public static final double ROBOT_MASS_KG = 68.4; 
    public static final double WHEEL_COF = 1.1;
    private static final PIDController rotationalPid = new PIDController(6.2, 0, 0);
    
    //Swerve drive sime
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
        .withCustomModuleTranslations(getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        // TODO: .withBumperSize(null, null)
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX44Foc(1),
            TunerConstants.FrontLeft.DriveMotorGearRatio, 
            TunerConstants.FrontLeft.SteerMotorGearRatio, 
            Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage), 
            Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage), 
            Meters.of(TunerConstants.FrontLeft.WheelRadius), 
            KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia), 
            WHEEL_COF));
            
    static final Lock odometryLock = new ReentrantLock();
    private final Gyro gyro;
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysId;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation3d robotOrientation = new Rotation3d();
    private SwerveModulePosition[] lastModulePositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private final SwerveDrivePoseEstimator3d poseEstimator = new SwerveDrivePoseEstimator3d(
        kinematics, robotOrientation, lastModulePositions, new Pose3d());

    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO,
        Consumer<Pose2d> resetSimulationPoseCallBack
    ) {
        this.gyro = new Gyro(gyroIO);
        modules[0] = new Module(flModuleIO, 1, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 2, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 3, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 4, TunerConstants.BackRight);
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
        PhoenixOdometryThread.getInstance().start();

        Pathfinding.setPathfinder(new LocalADStarAK());
        //If we ever use pathplanner again...
        // PathPlannerLogging.setLogActivePathCallback((activePath) -> {
        //     Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        // });
        // PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
        //     Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        // });
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())), 
            new SysIdRoutine.Mechanism(volts -> runCharacterization(volts.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // prevent updates in odometry while reading data
        gyro.periodic();
        for(Module m : modules) {
            m.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (Module m : modules) {
                m.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // update Odometry
        double[] timestamps = modules[0].getOdometryTimestamps(); //they are all together
        int sampleCount = timestamps.length;
        for(int i = 0; i < sampleCount; i++) {
            //read wheels poses and anl for each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] deltaPositions = new SwerveModulePosition[4];
            for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                deltaPositions[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            //update robot Rotation
            if(gyro.isGyroConnected()) {
                //use real gyro
                robotOrientation = gyro.getRobotOreintation(); // gyro.getTobotOreintation(i)? at some point;
            } else {    
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(deltaPositions);
               robotOrientation = robotOrientation.plus(new Rotation3d(Rotation2d.fromRadians(twist.dtheta)));
            }

            //apply odometry
            poseEstimator.updateWithTime(timestamps[i], robotOrientation, modulePositions);
        }
    }

    /** This is the method to run the drivetrain. It takes  speed and runs it. */
    public void runSpeeds(ChassisSpeeds speeds) {
        //find module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, TunerConstants.kSpeedAt12Volts);

        //Log unoptomized Setpoints
        Logger.recordOutput("Drive/SwerveStates/Unoptomized/Setpoints", desiredStates);
        Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", speeds);

        // Optomize and run modules
        for(int i = 0; i < 4; i++) {
            modules[i].runSetpoint(desiredStates[i]);
        }

        //Log optomized setpoints
        Logger.recordOutput("Drive/SwerveStates/Optomized/Setpoints", desiredStates);
    }

    /** runs the drive at speed */
    public void stop() {
        runSpeeds(new ChassisSpeeds());
    }

    /** xStop the drive, returns to regua positions once another speed is applied */
    public void xStop() {
        Rotation2d[] mHeadings = new Rotation2d[4];
        for(int i = 0; i < 4; i++) {
            mHeadings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(mHeadings);
        stop();
    }

    /** returns the current SwerveModulePositions of the modules */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] poses = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            poses[i] = modules[i].getPosition();
        }
        return poses;
    }

    /** returns the measured Rotation3d of the drive */
    public Rotation3d getRotation() {
        return getPose().getRotation();
    }

    /** resets the current pose to a pose2d */
    public void resetPose(Pose2d resetPose) {
        resetSimulationPoseCallBack.accept(resetPose);
        poseEstimator.resetPosition(getRotation(), getModulePositions(), new Pose3d(resetPose));
    }

    /** return the pose3d of the drive */
    @AutoLogOutput(key = "Drive/Odometry")
    public Pose3d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** returns the current module states of the drive */
    @AutoLogOutput(key = "Drive/SwerveStates/Measure")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** returns the desired module states of the drive */
    private SwerveModuleState[] getDesiredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = modules[i].getDesiredState();
        }
        return states;
    }

    /** retruns the current FO chassis speeds measured by the drive. */
    @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
    public ChassisSpeeds getSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getRotation().toRotation2d());
    }

    /** returns the current FO chassis speeds desired by the drive */
    public ChassisSpeeds getDesiredSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getDesiredModuleStates()), getRotation().toRotation2d());
    }

    public DoubleSupplier getDistanceToPose(Supplier<Pose2d> pose) {
        return () -> pose.get().getTranslation().getDistance(getPose().toPose2d().getTranslation());
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose3d visionRobotPoseMeters, double timestampSeconds, Matrix<N4, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    // All the characterization yada yada is down here.

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }
    
    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }
    
    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
    
    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    public static class DriveCommands {
        public static Command joystickDrive(Drive drive, DoubleSupplier y, DoubleSupplier x, DoubleSupplier theta) {
            return Commands.run(
                () -> drive.runSpeeds(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        -y.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(), //joystick y is forward (x) direction Field Relative
                        -x.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(), //joystick x is left/right (y) direction Field Relative
                        -theta.getAsDouble() * 4, 
                        isRedSide() ? drive.getRotation().toRotation2d().plus(Rotation2d.k180deg) : drive.getRotation().toRotation2d()
                    )
                ),
                drive
            );
        }
    }
}
