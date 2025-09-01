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
        
        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            constElevator.elevatorGearing,
            constElevator.carriageMass,
            constElevator.elevatorDrumRadius,
            constElevator.minElevatorHeightMeters,
            constElevator.maxElevatorHeightMeters,
            true,
            constElevator.minElevatorHeightMeters);

        Mechanism2d mech2d = new Mechanism2d(3, 3);
        MechanismRoot2d root = mech2d.getRoot("ElevatorRoot", 2, 0);
        carriage = root.append(new MechanismLigament2d("Carriage", constElevator.minElevatorHeightMeters, 90));
        SmartDashboard.putData("ElevatorSim", mech2d);

        simState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);
        simState.setRotorVelocity(0);
        targetMeters = elevatorSim.getPositionMeters();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorSim.setInputVoltage(simState.getMotorVoltage());
        elevatorSim.update(constElevator.simulationTick);

        simState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);
        simState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * constElevator.ROTATIONS_PER_METER);

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
}
