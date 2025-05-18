package frc.robot.subsystems.armSubsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constArm;

public class Arm extends SubsystemBase {
    TalonFX shoulderMotor;
    PositionVoltage positionRequest;
    VoltageOut voltageRequest = new VoltageOut(0);
    MotionMagicExpoVoltage motionRequest;
    double targetAngle = 0;
    double targetPosition = 0;
    ArmSim armSim;

    public Arm() {
        shoulderMotor = new TalonFX(0);

        voltageRequest = new VoltageOut(0);
        motionRequest = new MotionMagicExpoVoltage(0);

        shoulderMotor.getConfigurator().apply(constArm.shoulder.SHOULDER_CONFIG);

        armSim = new ArmSim(shoulderMotor);
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        targetPosition = Units.degreesToRotations(angle);
        shoulderMotor.setControl(motionRequest.withPosition(Units.degreesToRotations(angle)));

    }

    @Override
    public void periodic() {
        SmartDashboard.putData(shoulderMotor);
        SmartDashboard.putNumber("motor voltage", shoulderMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("target angle", targetAngle);
        SmartDashboard.putNumber("target position", targetPosition);

    }

}
