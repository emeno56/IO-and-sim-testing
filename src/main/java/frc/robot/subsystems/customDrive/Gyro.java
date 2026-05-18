package frc.robot.subsystems.customDrive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Gyro {
    private final GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    private final Alert gyroDisconnect;

    public Gyro(GyroIO io) {
        this.io = io;
        this.gyroDisconnect = new Alert("Pigeon hs disconnected!", AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Gyro", inputs);
        gyroDisconnect.set(!inputs.connected);
    }

    public boolean isGyroConnected() {
        return inputs.connected;
    }

    public Rotation2d[] getOdometryRollPosition() {
        return inputs.odometryRollPositions;
    }

    public Rotation2d[] getOdometryPitchPosition() {
        return inputs.odometryPitchPositions;
    }

    public Rotation2d[] getOdometryYawPosition() {
        return inputs.odometryYawPositions;
    }

    public Rotation3d getRobotOreintation() {
        return new Rotation3d(
            inputs.roll.getMeasure(), 
            inputs.pitch.getMeasure(), 
            inputs.yaw.getMeasure());
    }

}
