package frc.robot.subsystems.customDrive;

import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.util.PhoenixUtil;

public class ModuleIOKrakenSim extends ModuleIOKraken {
    SwerveModuleSimulation moduleSim;

    public ModuleIOKrakenSim(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants, SwerveModuleSimulation sim) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(constants));
        //create the simulation
        this.moduleSim = sim;

        //use simulated motors
        moduleSim.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(drive));
        moduleSim.useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(steer, absoluteEncoder));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update timestamps
        inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

        inputs.odometryDrivePositionsRad = Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();

        inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
    }
    
}
