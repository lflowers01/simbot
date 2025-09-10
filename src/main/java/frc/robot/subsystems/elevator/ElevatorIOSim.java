package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.constElevator;

public class ElevatorIOSim extends ElevatorIO.TalonFXBase {
    private final TalonFXSimState simState;
    private final ElevatorSim elevatorSim;
    private final MechanismLigament2d carriage;

    public ElevatorIOSim() {
        super();

        simState = leadMotor.getSimState();

        // Create the elevator simulation
        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2), // 2 Kraken X60 motors
                constElevator.gearing,
                constElevator.carriageMass,
                constElevator.drumRadius,
                constElevator.minHeightMeters,
                constElevator.maxHeightMeters,
                true, // Simulate gravity
                constElevator.minHeightMeters);

        Mechanism2d mech2d = new Mechanism2d(1, constElevator.maxHeightMeters + 0.2);
        MechanismRoot2d root = mech2d.getRoot("ElevatorRoot", constElevator.horizontalOffset,
                constElevator.verticalOffset);
        carriage = root.append(new MechanismLigament2d("Carriage", constElevator.minHeightMeters, 90,
                constElevator.lineWidth,
                constElevator.color));
        SmartDashboard.putData("ElevatorSim", mech2d);

        simState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.rotationsPerMeter);
        simState.setRotorVelocity(0);
        targetMeters = elevatorSim.getPositionMeters();

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorSim.setInputVoltage(simState.getMotorVoltage());
        elevatorSim.update(constElevator.simulationTick);

        simState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.rotationsPerMeter);
        simState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * constElevator.rotationsPerMeter);

        super.updateInputs(inputs);

        carriage.setLength(elevatorSim.getPositionMeters());
        SmartDashboard.putNumber("Elevator Height (m)", elevatorSim.getPositionMeters());
        SmartDashboard.putNumber("Elevator Goal (m)", targetMeters);
    }

    @Override
    protected void updateInterfaceInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    public void setHeight(double meters) {
        super.setHeight(meters);
        elevatorSim.setInput(leadMotor.getMotorVoltage().getValueAsDouble());
    }
}
