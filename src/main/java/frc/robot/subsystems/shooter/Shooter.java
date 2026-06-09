package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static final Pose3d HOOD_OFFSET = new Pose3d(0.287, 0, 0.53, new Rotation3d(0, Units.degreesToRadians(-10), 0));
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

    public Command setShot(double mps, double angle) {
        return run(() ->io.setShooterState(mps, angle));
    }
}
