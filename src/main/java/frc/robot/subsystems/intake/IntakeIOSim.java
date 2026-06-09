package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.generated.TunerConstants;

public class IntakeIOSim extends IntakeIOKraken {
    private final IntakeSimulation intake;
    private static final double ROLLER_RADIUS_METERS = 
        Units.inchesToMeters(inPerRot / (2 * Math.PI));
    private final ElevatorSim extensionSim = new ElevatorSim(
        DCMotor.getKrakenX44Foc(2),
        EXTENSION_GEARING,
        5,
        ROLLER_RADIUS_METERS,
        0.0,
        Units.inchesToMeters(10.5),
        false,
        0.0
    );
    private final DCMotorSim rollerModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.004, ROLLER_GEARING
        ),
        DCMotor.getKrakenX60Foc(1)
    );
    private final AbstractDriveTrainSimulation driveSim;
    
    private TalonFXSimState leftMotorSim;
    private TalonFXSimState rightMotorSim;
    private TalonFXSimState rollerSim;

    public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
        this.driveSim = driveSim;
        intake = IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, 
            Meters.of(Math.abs(TunerConstants.BackLeft.LocationY - TunerConstants.BackRight.LocationY)), 
            Inches.of(12), 
            IntakeSide.BACK, 
            88);

        leftMotorSim = leftMotor.getSimState();
        rightMotorSim = rightMotor.getSimState();
        rollerSim = roller.getSimState();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        extensionPeriodic();
        rollerPeriodic();

        super.updateInputs(inputs);
        inputs.leftIsConnected = true;
        inputs.rightIsConnected = true;
        inputs.rollerIsConnected = true;
        if(!inputs.isRunning) {
            intake.stopIntake();
        } else {
            intake.startIntake();
        }

        Logger.recordOutput("GamePieces/Fuel", getFuelPoses(
            intake.getGamePiecesAmount(),
            inputs.extendDistance
        ));
    }

    private void rollerPeriodic() {
        rollerSim = roller.getSimState();

        rollerSim.setSupplyVoltage(12);
        
        double rollerVolt = rollerSim.getMotorVoltage();

        rollerModel.setInputVoltage(rollerVolt);

        rollerModel.update(0.020);

        rollerSim.setRawRotorPosition(rollerModel.getAngularPositionRotations() * (ROLLER_GEARING));
        rollerSim.setRotorVelocity(rollerModel.getAngularVelocity().times(ROLLER_GEARING));

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

        double rotorPosition = mechanismRotations * EXTENSION_GEARING;
        double rotorVelocity = mechanismVelocity  * EXTENSION_GEARING;

        leftMotorSim.setRawRotorPosition(rotorPosition);
        leftMotorSim.setRotorVelocity(rotorVelocity);
        rightMotorSim.setRawRotorPosition(-rotorPosition);
        rightMotorSim.setRotorVelocity(rotorVelocity);
    }

    private final double ballDiameter = Units.inchesToMeters(5.91);
    private final double hopperX = -0.1;
    private final double hopperY = 0.0;
    private final double BALL_HEIGHT_METERS = Units.inchesToMeters(3.0);

    private Pose3d[] getFuelPoses(int count, double extendDistanceInches) {
        if (count == 0) return new Pose3d[0];

        Pose3d[] poses = new Pose3d[count];

        Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
        Pose3d robotPose3d = new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );

        int ballsPerRow = 6;
        int ballsPerCol = 4;
        int ballsPerLayer = ballsPerRow * ballsPerCol;

        int placed = 0;
        int slot = 0;

        while (placed < count) {
            int layer     = slot / ballsPerLayer;
            int remainder = slot % ballsPerLayer;
            int row       = remainder / ballsPerCol;
            int col       = remainder % ballsPerCol;

            slot++;

            // Skip front row (row == 0) top 2 layers (layer >= 2)
            if (row == 0 && layer >= 2) continue;

            // Safety — grid is full
            if (layer >= 4) break;

            double x = hopperX + (Units.inchesToMeters(extendDistanceInches) / 2.0)
                    - (row * ballDiameter)
                    - ballDiameter / 2
                    + (1.5 * ballDiameter)
                    + Units.inchesToMeters(1);

            double y = hopperY + ((col - 1.5) * ballDiameter);

            double z = BALL_HEIGHT_METERS + Units.inchesToMeters(3.0) + (layer * ballDiameter);

            poses[placed] = robotPose3d.plus(
                new Transform3d(x, y, z, new Rotation3d())
            );

            placed++;
        }

        return poses;
    }
}
