package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.getHubLocation;
import static frc.robot.Constants.isRedSide;

import java.util.Set;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.customDrive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LaunchCalculator.LaunchParameters;
import frc.robot.util.FuelUtil;

public class Shooter extends SubsystemBase {
    public static final Pose3d HOOD_OFFSET = new Pose3d(0.287, 0, 0.53, new Rotation3d(0, Units.degreesToRadians(-10), 0));
    public static final double kShooterOffsetX = 0.2; //??? in meters
    public static final double kShooterOffsetZ = 0.6; //??? meters
    public static final double kShooterOffsetY = 0;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Alert m1Dis;
    private final Alert m2Dis;
    private final Alert m3Dis;
    private final Alert m4Dis;
    private final Alert hoodDis;

    public Shooter(ShooterIO io) {
        this.io = io;
        m1Dis = new Alert("Shooter motor one has disconnected", AlertType.kError);
        m2Dis = new Alert("Shooter motor two has disconnected", AlertType.kError);
        m3Dis = new Alert("Shooter motor three has disconnected", AlertType.kError);
        m4Dis = new Alert("Shooter motor four has disconnected", AlertType.kError);
        hoodDis = new Alert("Shooter hood motor has disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Component", HOOD_OFFSET.plus(new Transform3d(0, 0, 0, new Rotation3d(0, Units.degreesToRadians(inputs.hoodPosition), 0))));//.minus(new Pose3d(0, 0, 0, new Rotation3d(0, 0, -inputs.hoodPosition))));
        m1Dis.set(!inputs.motor1IsConnected);
        m2Dis.set(!inputs.motor2IsConnected);
        m3Dis.set(!inputs.motor3IsConnected);
        m4Dis.set(!inputs.motor4IsConnected);
        hoodDis.set(!inputs.hoodIsConnected);
    }

    public Command defaultCommand() {
        return run(() -> io.stop());
    }

    public void setShot(LaunchParameters parameters) {
        io.setShooterState(parameters.mps, parameters.angle);
    }

    public LaunchParameters getCurrentLaunchParameters() {
        return new LaunchParameters(getFuelExitVelocity().getAsDouble(), getExitAngle().getAsDouble());
    }

    public DoubleSupplier getFuelExitVelocity() {
        return () -> inputs.fuelExitVelocityMPS;
    }

    public DoubleSupplier getExitAngle() {
        return () -> inputs.hoodPosition;
    }

    public static class ShooterCommands {
        public static Command shoot(Shooter shooter, Intake intake, Drive drive, IntakeSimulation intakeSim) {
            return Commands.defer(() -> { 
                return
                    Commands.run(() -> {
                        LaunchParameters parameters = LaunchCalculator.calculateBestParameters(
                            drive.getDistanceToPose(() -> getHubLocation()).getAsDouble()
                        );
                        shooter.setShot(parameters);
                        if(shooter.getFuelExitVelocity().getAsDouble() > parameters.mps - 0.2) {
                            if(Constants.currentMode == Constants.Mode.SIM) {
                                if(intakeSim.obtainGamePieceFromIntake()){
                                    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                                        drive.getPose().toPose2d().getTranslation(), 
                                        new Translation2d(kShooterOffsetX, Math.random() * 0.6 - 0.3), 
                                        drive.getSpeeds(), 
                                        drive.getRotation().toRotation2d(), 
                                        Meters.of(kShooterOffsetZ), 
                                        MetersPerSecond.of(shooter.getFuelExitVelocity().getAsDouble()), 
                                        Degrees.of(90 - shooter.getExitAngle().getAsDouble()));

                                        fuelOnFly
                                            .withTargetPosition(() -> new Translation3d(11.938, 4.035, Units.inchesToMeters(72)))
                                            .withTargetTolerance(new Translation3d(Units.inchesToMeters(41.7 / 2.0), Units.inchesToMeters(41.7 / 2.0), 0.1))
                                            .withProjectileTrajectoryDisplayCallBack(
                                                (pose3ds) -> Logger.recordOutput("Fuel/Shot/Success", pose3ds.toArray(Pose3d[]::new)),
                                                // Callback for when the fuel will eventually miss the target, or if no target is configured
                                                (pose3ds) -> Logger.recordOutput("Fuel/Shot/Fail", pose3ds.toArray(Pose3d[]::new))
                                            )
                                            .setHitTargetCallBack(() -> FuelUtil.addHubExitFuel(isRedSide()));;
                                    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
                                }
                            }
                            intake.closeIntake();
                        }
                    });
                }, Set.of()).repeatedly();
        }
    }
}
