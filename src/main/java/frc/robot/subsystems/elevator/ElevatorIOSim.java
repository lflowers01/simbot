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
        super(1);

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

        // Initialize sim rotor position/velocity using positionSign
        simState.setRawRotorPosition(positionSign * elevatorSim.getPositionMeters() * constElevator.rotationsPerMeter);
        simState.setRotorVelocity(0);
        targetMeters = elevatorSim.getPositionMeters();

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorSim.setInput(simState.getMotorVoltage());
        elevatorSim.update(constElevator.simulationTick);

        // Apply positionSign when writing back to Talon sim state
        simState.setRawRotorPosition(positionSign * elevatorSim.getPositionMeters() * constElevator.rotationsPerMeter);
        simState.setRotorVelocity(
                positionSign * elevatorSim.getVelocityMetersPerSecond() * constElevator.rotationsPerMeter);

        super.updateInputs(inputs);

        carriage.setLength(elevatorSim.getPositionMeters() + 1);
        SmartDashboard.putNumber("Elevator Height (m)", elevatorSim.getPositionMeters());
        SmartDashboard.putNumber("Elevator Goal (m)", targetMeters);
    }

    @Override
    protected void updateInterfaceInputs(ElevatorIOInputs inputs) {
        // Mirror real IO: read simulated Talon encoder values, apply positionSign, and
        // clamp.
        double posMeters = positionSign * leadMotor.getPosition().getValueAsDouble() / constElevator.rotationsPerMeter;
        double velMeters = positionSign * leadMotor.getVelocity().getValueAsDouble() / constElevator.rotationsPerMeter;

        double clampedPos = Math.max(constElevator.minHeightMeters, Math.min(constElevator.maxHeightMeters, posMeters));
        if (clampedPos != posMeters) {
            System.out.println(String.format("Elevator (SIM): sensor position %.3f m out of bounds, clamped to %.3f m",
                    posMeters, clampedPos));
        }

        inputs.positionMeters = clampedPos;
        inputs.velocityMetersPerSec = velMeters;
    }

    @Override
    public void setHeight(double meters) {
        super.setHeight(meters);
        elevatorSim.setInput(leadMotor.getMotorVoltage().getValueAsDouble());
    }
}
