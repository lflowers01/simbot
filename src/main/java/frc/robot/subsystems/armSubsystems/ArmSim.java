package frc.robot.subsystems.armSubsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSim extends SubsystemBase {

    public final TalonFXSimState shoulderMotorSim;
    private final SingleJointedArmSim shoulderJointSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            Constants.kArmReduction,
            SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
            Constants.kArmLength,
            Constants.kMinAngleRads,
            Constants.kMaxAngleRads,
            true,
            0,
            Constants.kArmEncoderDistPerPulse,
            0.0 // Add noise with a std-dev of 1 tick

    );
    private final Mechanism2d mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armBase = armPivot.append(new MechanismLigament2d("L0", 30, -90));
    private final MechanismLigament2d upperArm;
    TalonFX simMotor;

    public ArmSim(TalonFX motor) {
        simMotor = motor;
        shoulderMotorSim = simMotor.getSimState();
        upperArm = armPivot.append(
                new MechanismLigament2d(
                        "Arm",
                        30,
                        Units.rotationsToDegrees(simMotor.getPosition().getValueAsDouble()),
                        6,
                        new Color8Bit(Color.kYellow)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Arm Sim", mech2d);
        shoulderMotorSim.setSupplyVoltage(simMotor.getSupplyVoltage().getValue());

        shoulderJointSim.setInput(shoulderMotorSim.getMotorVoltage());
        shoulderJointSim.update(0.02);
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(shoulderJointSim.getCurrentDrawAmps()));

        upperArm.setAngle(Units.radiansToDegrees(shoulderJointSim.getAngleRads()));
        
    }
}
