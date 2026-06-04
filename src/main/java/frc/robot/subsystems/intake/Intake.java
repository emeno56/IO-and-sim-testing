package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
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
}
