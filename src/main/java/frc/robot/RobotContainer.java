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

import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.customDrive.Drive.DriveCommands;
import frc.robot.subsystems.vision.*;
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
    private final Drive drive;
    private final Vision vision;

    private SwerveDriveSimulation driveSimulation = null;

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
                vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                );
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
        joystick.x().onTrue(Commands.run(() -> drive.xStop(), drive));
    } 

    public void resetSimulationFuel() {
        if(Constants.currentMode != Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, Rotation2d.kZero));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if(Constants.currentMode != Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Pose3d[] fuel = SimulatedArena.getInstance().getGamePiecesArrayByType(
            "Fuel"
        );

        Logger.recordOutput("Simulation/Pose", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel", fuel);
    }
}