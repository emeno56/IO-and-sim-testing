package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import frc.robot.generated.TunerConstants;

public class IntakeIOKrakenSim extends IntakeIOKraken {
    private final IntakeSimulation intake;

    public IntakeIOKrakenSim(AbstractDriveTrainSimulation driveSim) {
        intake = IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, 
            Meters.of(Math.abs(TunerConstants.BackLeft.LocationY - TunerConstants.BackRight.LocationY)), 
            Inches.of(12), 
            IntakeSide.BACK, 
            60);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.leftIsConnected = true;
        inputs.rightIsConnected = true;
        inputs.rollerIsConnected = true;
        if(!inputs.isRunning) {
            intake.stopIntake();
        }
    }

    @Override
    public void deployIntake() {
        super.deployIntake();
        intake.startIntake();
    }
}
