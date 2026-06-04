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
import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.util.bump.Bump.BUMPS;

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
import frc.robot.subsystems.intake.IntakeIOKrakenSim;
import frc.robot.subsystems.customDrive.Drive.DriveCommands;
import frc.robot.subsystems.vision.*;
import frc.robot.util.FuelUtil;
import frc.robot.util.bump.Bump;
import frc.robot.util.bump.BumpUtil;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.json.simple.ItemList;
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
                bump = new BumpUtil(driveSimulation::getSimulatedDriveTrainPose, this::getBumpPose);
                vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                );
                intake = new Intake(new IntakeIOKrakenSim(driveSimulation));
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
        joystick.x().onTrue(Commands.run(() -> drive.xStop(), drive));
        joystick.leftTrigger().whileTrue(intake.runIntake());
    } 

    public void resetSimulationFuel() {
        if(Constants.currentMode != Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(FIELD_LENGTH_M - 3, 3, Rotation2d.kZero));
        FuelUtil.fullFieldReset();
    }

    public void updateSimulation() {
        if(Constants.currentMode != Mode.SIM) return;
        bump.updateChecks();

        SimulatedArena.getInstance().simulationPeriodic();
        Pose3d[] fuel = SimulatedArena.getInstance().getGamePiecesArrayByType(
            "Fuel"
        );

        Pose3d bumpRobotPose = bump.bumpVisualizePose();

        for(Bump b : BUMPS) {
            if(wasOnBump && !b.isInBump(bumpRobotPose.toPose2d())) {
                driveSimulation.setSimulationWorldPose(bumpRobotPose.toPose2d());
                wasOnBump = false;
                break;
            }
        }
        Logger.recordOutput("Simulation/Pose3d", bumpRobotPose);
        Logger.recordOutput("Simulation/Sim Pose", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel", fuel);
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