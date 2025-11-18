package frc.robot.subsystems.elevator;

import frc.robot.Constants.constElevator;

public class ElevatorIOReal extends ElevatorIO.TalonFXBase {

    public ElevatorIOReal() {
        // Negative motor duty moves elevator up on the real hardware:
        super(-1);
    }

    @Override
    protected void updateInterfaceInputs(ElevatorIOInputs inputs) {
        // Read raw encoder values and apply sign so reported meters are positive
        // upwards
        double posMeters = positionSign * leadMotor.getPosition().getValueAsDouble() / constElevator.rotationsPerMeter;
        double velMeters = positionSign * leadMotor.getVelocity().getValueAsDouble() / constElevator.rotationsPerMeter;

        // Clamp to configured elevator bounds to avoid weird/invalid readings from
        // confusing callers
        double clampedPos = Math.max(constElevator.minHeightMeters, Math.min(constElevator.maxHeightMeters, posMeters));
        if (clampedPos != posMeters) {
            System.out.println(String.format("Elevator: sensor position %.3f m out of bounds, clamped to %.3f m",
                    posMeters, clampedPos));
        }

        inputs.positionMeters = clampedPos;
        inputs.velocityMetersPerSec = velMeters;
    }
}
