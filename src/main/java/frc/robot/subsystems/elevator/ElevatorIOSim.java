package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.constElevator;

public class ElevatorIOSim implements ElevatorIO {
    // Declare fields (but don’t initialize yet)
    private final TalonFX leadMotor;
    private final TalonFXSimState simState;
    private final ElevatorSim elevatorSim;

    private final Mechanism2d mech2d;
    private final MechanismRoot2d root;
    private final MechanismLigament2d carriage;
    final TalonFXConfiguration leadConfig;
    final MotionMagicVoltage control;
    private double targetMeters;

    public ElevatorIOSim() {
        // Motor + sim setup
        leadMotor = new TalonFX(constElevator.LEAD_MOTOR_ID);
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

        // Mechanism2d setup
        mech2d = new Mechanism2d(3, 3);
        root = mech2d.getRoot("ElevatorRoot", 2, 0);
        carriage = root.append(new MechanismLigament2d("Carriage", 0.1, 90));

        leadConfig = new TalonFXConfiguration();

        leadConfig.Slot0.kG = constElevator.KG;
        leadConfig.Slot0.kS = constElevator.KS;
        leadConfig.Slot0.kV = constElevator.KV;
        // leadConfig.Slot0.KF = constElevator.KF;
        leadConfig.Slot0.kA = constElevator.KA;
        leadConfig.Slot0.kP = constElevator.KP;
        leadConfig.Slot0.kI = constElevator.KI;
        leadConfig.Slot0.kD = constElevator.KD;
        // Sync sim position first
        
        simState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);
        simState.setRotorVelocity(0);

        // Initialize Motion Magic voltage AFTER rotor is synced
        control = new MotionMagicVoltage(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);//.withFeedForward(constElevator.KG);

        MotionMagicConfigs elevatorMotion = leadConfig.MotionMagic;
        elevatorMotion.MotionMagicCruiseVelocity = constElevator.motionVelocity * constElevator.ROTATIONS_PER_METER;
        elevatorMotion.MotionMagicAcceleration = constElevator.motionAcceleration * constElevator.ROTATIONS_PER_METER;
        elevatorMotion.MotionMagicExpo_kA = constElevator.EXPO_KA;
        elevatorMotion.MotionMagicExpo_kV = constElevator.EXPO_KV;
        
        leadMotor.getConfigurator().apply(leadConfig);

        // Initialize state

        targetMeters = elevatorSim.getPositionMeters();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update physics
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorSim.setInputVoltage(simState.getMotorVoltage());
        elevatorSim.update(0.02);

        simState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);
        simState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * constElevator.ROTATIONS_PER_METER);

        // Fill inputs
        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.appliedVolts = leadMotor.getSupplyVoltage().getValueAsDouble();
        inputs.currentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();
        inputs.targetMeters = targetMeters;

        // Update Mechanism2d visualization
        carriage.setLength(elevatorSim.getPositionMeters());
        SmartDashboard.putData("ElevatorSim", mech2d);
        SmartDashboard.putNumber("Elevator Height (m)", elevatorSim.getPositionMeters());
        SmartDashboard.putNumber("Elevator Goal (m)", targetMeters);
        // System.out.println("Elevator Height (m): " +
        // elevatorSim.getPositionMeters());

    }

    @Override
    public void setHeight(double meters) {
        targetMeters = meters;
        // Motion Magic handles kG internally
        leadMotor.setControl(control.withPosition(meters * constElevator.ROTATIONS_PER_METER));
    }

}
