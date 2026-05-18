package frc.robot.subsystems.customDrive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final Alert driveDisconnect;
    private final Alert steerDisconnect;
    private final Alert encoderDisconnect;

    private SwerveModulePosition[] odometryPosition = new SwerveModulePosition[] {};
        public Module(ModuleIO io, int index, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
            this.io = io;
            this.index = index;
            this.constants = constants;
    
            driveDisconnect = new Alert("Drive motor on module " + Integer.toString(index) + " has disconnected", AlertType.kError);
            steerDisconnect = new Alert("Steer motor on module " + Integer.toString(index) + " has disconnected", AlertType.kError);
            encoderDisconnect = new Alert("CANcoder on module " + Integer.toString(index) + " has disconnected", AlertType.kError);
        }
    
        public void periodic() {
            io.updateInputs(inputs);
            Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    
        // Find odometry positions
        // IDK what this does, but it seems important
        int sampleCount = inputs.odometryTimestamps.length;
        odometryPosition = new SwerveModulePosition[sampleCount];
        for(int i = 0; i < inputs.odometryTimestamps.length; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i];
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPosition[i] = new SwerveModulePosition(positionMeters, angle);
        }

        //update alerts
        driveDisconnect.set(!inputs.driveConnected);
        steerDisconnect.set(!inputs.steerConnected);
        encoderDisconnect.set(!inputs.absoluteEncoderConnected);
    }

    /** Set the module to the provided module state */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize
        state.optimize(getAngle().get());
        state.cosineScale(getAngle().get());

        //apply state
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setSteerPosition(state.angle);
    }

    /** return the angle of the module */
    public Supplier<Rotation2d> getAngle() {
        return () -> inputs.steerPosition;
    }

    /** return the current state of the module */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMps(), getAngle().get());
    }

    /** returns the module position (steer anlge and drive position) */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle().get());
    }

    /** apply 0 to all aspects of the module */
    public void stop() {
        io.setDriveOpenLoop(0);
        io.setSteerOpenLoop(0);
    }

    /** returns the drive motor's position in meters */
    public double getDrivePosition() {
        return inputs.drivePosition * constants.WheelRadius;
    }

    /** returns the drive velocity in m/s */
    public double getDriveVelocityMps() {
        return inputs.driveVelocityRadPerSec * constants.WheelRadius;
    }

    /** retruns module positions from this cycle */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPosition;
    }

    /** returns timestamps of the positions recieved this cycle */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** returns the drive position in radians */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePosition;
    }

    /** returns the drive velocity in rot/s */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

      /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setSteerPosition(Rotation2d.kZero);
    }
}
