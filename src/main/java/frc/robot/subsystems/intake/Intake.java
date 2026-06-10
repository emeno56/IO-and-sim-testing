package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static final Pose3d INTAKE_OFFSET = new Pose3d(-(0.055 + 0.042), 0, 0.425 - 0.235, Rotation3d.kZero);
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final Alert leftDisconnect;
    private final Alert rightDisconnect;
    private final Alert rollerDisconnect;

    public Intake(IntakeIO io) {
        this.io = io;
        leftDisconnect = new Alert("Left extension motor has disconnected", AlertType.kError);
        rightDisconnect = new Alert("Right extension motor has disconnected", AlertType.kError);
        rollerDisconnect = new Alert("Intake rollers have disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/Component", INTAKE_OFFSET.minus(new Pose3d(Units.inchesToMeters(inputs.extendDistance), 0, 0, Rotation3d.kZero)));
        leftDisconnect.set(!inputs.leftIsConnected);
        rightDisconnect.set(!inputs.rightIsConnected);
        rollerDisconnect.set(!inputs.rollerIsConnected);
    }

    public Command runIntake() {
        return run(() -> io.deployIntake());
    }

    public Command defaultCommand() {
        return run(() -> io.stopIntake());
    }

    public Command agitateIntake() {
        return run(() -> {
            double target = (Timer.getFPGATimestamp() % 1.0) < 0.5 ? 7.5 : 10.5;
            io.setIntakeState(target, 50);
        });
    }

    public int getFuelCount() {
        return io.getFuelCount();
    }

    public double getExtendDistance() {
        return inputs.extendDistance;
    }
}
