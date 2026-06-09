package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class ShooterIOSim extends ShooterIOKraken {
    private final FlywheelSim drumModel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(4), 
            0.15, 
            0.25), 
        DCMotor.getKrakenX60Foc(4)
    );
    private final DCMotorSim hoodModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), 
            0.0196, 
            HOOD_GEARING), 
        DCMotor.getKrakenX44Foc(1));
    private TalonFXSimState m1Sim;
    private TalonFXSimState m2Sim;
    private TalonFXSimState m3Sim;
    private TalonFXSimState m4Sim;
    private TalonFXSimState hoodSim;

    private TalonFXSimState[] mSims = new TalonFXSimState[4];

    public ShooterIOSim() {
        m1Sim = motor1.getSimState();
        m2Sim = motor2.getSimState();
        m3Sim = motor3.getSimState();
        m4Sim = motor4.getSimState();
        hoodSim = hood.getSimState();

        mSims[0] = m1Sim;
        mSims[1] = m2Sim;
        mSims[2] = m3Sim;
        mSims[3] = m4Sim;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        hoodPeriodic();
        drumPeriodic();

        super.updateInputs(inputs);
    }

    private void hoodPeriodic() {
        hoodSim = hood.getSimState();

        hoodSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double hoodVolt = hoodSim.getMotorVoltage();

        hoodModel.setInputVoltage(hoodVolt);
        hoodModel.update(0.020);

        hoodSim.setRawRotorPosition(hoodModel.getAngularPositionRotations() * HOOD_GEARING);
        hoodSim.setRotorVelocity(hoodModel.getAngularVelocity().times(HOOD_GEARING));
    }

    private void drumPeriodic() {
        double volt = m1Sim.getMotorVoltage();
        drumModel.setInputVoltage(volt);
        drumModel.update(0.020);

        for(int i = 0; i < 4; ++i) {
            mSims[i] = drums[i].getSimState();

            mSims[i].setSupplyVoltage(RobotController.getBatteryVoltage());

            mSims[i].setRotorVelocity(drumModel.getAngularVelocity());
        }
    }
}
