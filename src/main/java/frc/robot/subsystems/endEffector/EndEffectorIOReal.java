package frc.robot.subsystems.endEffector;

import frc.robot.Constants.constEndEffector;

public class EndEffectorIOReal extends EndEffectorIO.TalonFXBase {

    public EndEffectorIOReal() {
        // Use 1 for normal motor direction (positive commands move in positive
        // direction)
        super(1);
    }

    @Override
    protected void updateInterfaceInputs(EndEffectorIOInputs inputs) {
        // Read raw motor positions and apply sign correction
        double position = positionSign * motor.getPosition().getValueAsDouble();
        double velocity = positionSign * motor.getVelocity().getValueAsDouble();

        // Clamp to configured position bounds to avoid weird/invalid readings
        double clampedPosition = Math.max(constEndEffector.minPosition,
                Math.min(constEndEffector.maxPosition, position));

        if (Math.abs(clampedPosition - position) > 0.01) { // Only warn if significantly out of bounds
            System.out.println(String.format("EndEffector: sensor position %.3f out of bounds, clamped to %.3f",
                    position, clampedPosition));
        }

        inputs.position = clampedPosition;
        inputs.velocityRotPerSec = velocity;
    }
}
