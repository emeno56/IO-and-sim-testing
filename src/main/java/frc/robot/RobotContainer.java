// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.FIELD_LENGTH_M;
import static frc.robot.Constants.getHubLocation;
import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.util.bump.Bump.BUMPS;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.customDrive.Drive;
import frc.robot.subsystems.customDrive.GyroIO;
import frc.robot.subsystems.customDrive.GyroIOPigeon2Real;
import frc.robot.subsystems.customDrive.GyroIOSim;
import frc.robot.subsystems.customDrive.ModuleIO;
import frc.robot.subsystems.customDrive.ModuleIOKrakenReal;
import frc.robot.subsystems.customDrive.ModuleIOKrakenSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.Shooter.ShooterCommands;
import frc.robot.subsystems.customDrive.Drive.DriveCommands;
import frc.robot.subsystems.vision.*;
import frc.robot.util.FuelUtil;
import frc.robot.util.bump.Bump;
import frc.robot.util.bump.BumpUtil;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //Subsutems
    @SuppressWarnings({ "Unused", "unused" })
    private final Vision vision;
    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;

    //sim stuff
    private static final double BUMP_SPEED_SCALE_MIN = 0.2; // minimum speed multiplier
    private static final double BUMP_SPEED_SCALE_TIME = 0.75; // seconds to reach minimum
    private SwerveDriveSimulation driveSimulation = null;
    private BumpUtil bump = null;
    private boolean wasOnBump = false;
    private Pose2d bumpVisualizationPose = null;
    private double lastBumpUpdateTime = -1;
    private double bumpTimeSeconds = 0.0;

        //Controller
    private final CommandXboxController joystick = new CommandXboxController(0);
   
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2Real(), 
                    new ModuleIOKrakenReal(TunerConstants.FrontLeft), 
                    new ModuleIOKrakenReal(TunerConstants.FrontRight), 
                    new ModuleIOKrakenReal(TunerConstants.BackLeft), 
                    new ModuleIOKrakenReal(TunerConstants.BackRight), 
                    (pose) -> {});
                vision = new Vision(
                    drive, 
                    new VisionIOPhotonVision(camera0Name, robotToCamera0),
                    new VisionIOPhotonVision(camera1Name, robotToCamera1)
                );
                intake = new Intake(new IntakeIOKraken());
                shooter = new Shooter(new ShooterIOKraken());
                break;
            case SIM:
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d());
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()), 
                    new ModuleIOKrakenSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]), 
                    new ModuleIOKrakenSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]), 
                    new ModuleIOKrakenSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]), 
                    new ModuleIOKrakenSim(TunerConstants.BackRight, driveSimulation.getModules()[3]), 
                    driveSimulation::setSimulationWorldPose);
                drive.resetPose(driveSimulation.getSimulatedDriveTrainPose());
                bump = new BumpUtil(driveSimulation::getSimulatedDriveTrainPose, this::getBumpPose);
                vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                );
                intake = new Intake(new IntakeIOSim(driveSimulation));
                intake.setSim();
                shooter = new Shooter(new ShooterIOSim());
                break;
        default:
            //Replay Mode
            drive = new Drive(
                new GyroIO() {}, 
                new ModuleIO() {}, 
                new ModuleIO() {}, 
                new ModuleIO() {}, 
                new ModuleIO() {}, 
                (pose) -> {});
            vision = new Vision(
                drive, 
                new VisionIO() {},
                new VisionIO() {});
            intake = new Intake(new IntakeIO() {});
            shooter = new Shooter(new ShooterIO() {});
            break;
        }

        configureBindings();
    }

    public void configureBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
            drive, 
            joystick::getLeftY, 
            joystick::getLeftX, 
            joystick::getRightX));
        intake.setDefaultCommand(intake.defaultCommand());
        shooter.setDefaultCommand(shooter.defaultCommand());
        joystick.x().onTrue(Commands.run(() -> drive.xStop(), drive));
        joystick.leftTrigger().whileTrue(intake.runIntake());
        joystick.a().whileTrue(
            ShooterCommands.shoot(shooter, intake, drive, intake.getSim())
        );
        joystick.a().whileTrue(
            DriveCommands.pointToPoint(
                drive, 
                () -> getHubLocation().getTranslation(), 
                joystick::getLeftY, 
                joystick::getLeftX)
        );
    } 

    public void resetSimulationFuel() {
        if(Constants.currentMode != Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(15.89, 7.275, Rotation2d.fromDegrees(-141.25)));
        FuelUtil.fullFieldReset();
        // SimulatedArena.getInstance().resetFieldForAuto();; //TODO: Use this if your computer is really slow
    }

    public void updateSimulation() {
        if(Constants.currentMode != Mode.SIM) return;

        Pose3d[] fuel = SimulatedArena.getInstance().getGamePiecesArrayByType(
            "Fuel"
        );

        Pose3d bumpRobotPose = bump.bumpVisualizePose();

        if (bump.isOnBump() && bumpVisualizationPose != null) {
            for (Bump b : Bump.BUMPS) {
                if (b.isInBump(bumpVisualizationPose)) {
                    double minY = Math.min(b.getCorner1().getY(), b.getCorner2().getY()) + 0.065;
                    double maxY = Math.max(b.getCorner1().getY(), b.getCorner2().getY()) - 0.06;
                    double clampedY = MathUtil.clamp(bumpVisualizationPose.getY(), minY, maxY);
                    bumpVisualizationPose = new Pose2d(
                        bumpVisualizationPose.getX(), 
                        clampedY, 
                        bumpVisualizationPose.getRotation());
                    driveSimulation.setSimulationWorldPose(bumpVisualizationPose);
                    break;
                }
            }
        }

        bump.updateChecks(bumpVisualizationPose);

        SimulatedArena.getInstance().simulationPeriodic();

        for(Bump b : BUMPS) {
            if(wasOnBump && !b.isInBump(bumpRobotPose.toPose2d())) {
                driveSimulation.setSimulationWorldPose(bumpRobotPose.toPose2d());
                wasOnBump = false;
                break;
            }
        }
        
        Pose3d[] robotFuel = FuelUtil.getFuelPosesFromRobot(bumpRobotPose, intake.getFuelCount(), intake.getExtendDistance());

        Logger.recordOutput("Simulation/Robot/Real Sim Pose", bumpRobotPose);
        Logger.recordOutput("Simulation/Robot/Debug Sim Pose", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel/Fuel in robot", robotFuel);
        Logger.recordOutput("Simulation/Fuel/Fuel on field", fuel);
        Logger.recordOutput("Simulation/Fuel/Fuel on fly", LaunchCalculator.simulatePoses(bumpRobotPose, shooter.getCurrentLaunchParameters()));
    }

    private Pose2d getBumpPose() {
        if (bump != null && bump.isOnBump()) {
            wasOnBump = true;
            double now = Timer.getFPGATimestamp();

            if (bumpVisualizationPose == null) {
                bumpVisualizationPose = driveSimulation.getSimulatedDriveTrainPose();
                lastBumpUpdateTime = now;
                bumpTimeSeconds = 0.0;
                return bumpVisualizationPose;
            }

            double dt = now - lastBumpUpdateTime;
            lastBumpUpdateTime = now;

            boolean isOnPhysicalBump = false;
            for (Bump b : Bump.BUMPS) {
                if (b.isInPhysicalBump(driveSimulation.getSimulatedDriveTrainPose())) {
                    isOnPhysicalBump = true;
                    break;
                }
            }

            if (isOnPhysicalBump) {
                bumpTimeSeconds += dt;
            }

            double speedScale = isOnPhysicalBump ? Math.max(
                BUMP_SPEED_SCALE_MIN,
                1.0 - (bumpTimeSeconds / BUMP_SPEED_SCALE_TIME) * (1.0 - BUMP_SPEED_SCALE_MIN)
            ) : 1.0;

            ChassisSpeeds speeds = drive.getDesiredSpeeds();

            double newX = bumpVisualizationPose.getX() + speeds.vxMetersPerSecond * dt * speedScale;
            double newY = bumpVisualizationPose.getY() + speeds.vyMetersPerSecond * dt * speedScale;

            for (Bump b : Bump.BUMPS) {
                if (b.isInBump(bumpVisualizationPose)) {
                    double minY = Math.min(b.getCorner1().getY(), b.getCorner2().getY());
                    double maxY = Math.max(b.getCorner1().getY(), b.getCorner2().getY());
                    newY = MathUtil.clamp(newY, minY, maxY);
                    break;
                }
            }

            Rotation2d newHeading = bumpVisualizationPose.getRotation()
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

            bumpVisualizationPose = new Pose2d(newX, newY, newHeading);
            return bumpVisualizationPose;
        }

        bumpVisualizationPose = null;
        lastBumpUpdateTime = -1;
        bumpTimeSeconds = 0.0;
        return driveSimulation.getSimulatedDriveTrainPose();
    }
}