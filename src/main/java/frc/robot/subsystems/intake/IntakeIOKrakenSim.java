package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.generated.TunerConstants;

public class IntakeIOKrakenSim extends IntakeIOKraken {
    private final IntakeSimulation intake;
    private final double extendtionGearing = 46.0 / 11.0;
    private static final double DRUM_RADIUS_METERS = 
        Units.inchesToMeters(inPerRot / (2 * Math.PI));

    private final ElevatorSim extensionSim = new ElevatorSim(
        DCMotor.getKrakenX44Foc(2),
        extendtionGearing,
        1.5,
        DRUM_RADIUS_METERS,
        0.0,
        Units.inchesToMeters(12),
        false,
        0.0
    );

    private final DCMotorSim rollerModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.004, 24.0 / 11.0
        ),
        DCMotor.getKrakenX60Foc(1)
    );
    
    private TalonFXSimState leftMotorSim;
    private TalonFXSimState rightMotorSim;
    private TalonFXSimState rollerSim;



    public IntakeIOKrakenSim(AbstractDriveTrainSimulation driveSim) {
        intake = IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, 
            Meters.of(Math.abs(TunerConstants.BackLeft.LocationY - TunerConstants.BackRight.LocationY)), 
            Inches.of(12), 
            IntakeSide.BACK, 
            60);
        leftMotorSim = leftMotor.getSimState();
        rightMotorSim = rightMotor.getSimState();
        rollerSim = roller.getSimState();
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

        extensionPeriodic();
        rollerPeriodic();
    }

    @Override
    public void deployIntake() {
        super.deployIntake();
        intake.startIntake();
    }

    private void rollerPeriodic() {
        rollerSim = roller.getSimState();

        rollerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        double rollerVolt = rollerSim.getMotorVoltage();

        rollerModel.setInputVoltage(rollerVolt);

        rollerModel.update(0.020);

        rollerSim.setRawRotorPosition(rollerModel.getAngularPositionRotations() * (24.0 / 11.0));
        rollerSim.setRotorVelocity(rollerModel.getAngularVelocity().times(24.0 / 11.0));

    }
    private void extensionPeriodic() {
        leftMotorSim = leftMotor.getSimState();
        rightMotorSim = rightMotor.getSimState();

        leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        extensionSim.setInputVoltage(leftMotorSim.getMotorVoltage());
        extensionSim.update(0.020);

        // Meters → mechanism rotations → rotor rotations
        double mechanismRotations = extensionSim.getPositionMeters() / Units.inchesToMeters(inPerRot);
        double mechanismVelocity  = extensionSim.getVelocityMetersPerSecond() / Units.inchesToMeters(inPerRot);

        double rotorPosition = mechanismRotations * extendtionGearing;
        double rotorVelocity = mechanismVelocity  * extendtionGearing;

        leftMotorSim.setRawRotorPosition(rotorPosition);
        leftMotorSim.setRotorVelocity(rotorVelocity);
        rightMotorSim.setRawRotorPosition(-rotorPosition);
        rightMotorSim.setRotorVelocity(rotorVelocity);
    }
}
