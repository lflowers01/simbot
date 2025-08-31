package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class ElevatorOLD extends SubsystemBase{

    TalonFX leadMotor, followMotor;
    
    TalonFXConfiguration leadConfig, followConfig;
    final MotionMagicVoltage elevatorControl;

    // Simulation variables
    Mechanism2d elevatorBase;
    MechanismRoot2d elevatorRoot;
    MechanismLigament2d elevatorDraw;
    double elevatorMinimumLength;

    double elevatorTarget;

    TalonFXSimState leadMotorSimState;
    TalonFXSimState followMotorSimState;
    DCMotorSim leadMotorSim;
    DCMotorSim followMotorSim;

    ElevatorSim elevatorSim;

    public ElevatorOLD(double elevatorMinimumLength) {

        leadMotor = new TalonFX(constElevator.LEAD_MOTOR_ID);
        followMotor = new TalonFX(constElevator.FOLLOW_MOTOR_ID);
    
        followMotorSimState = followMotor.getSimState();
        leadMotorSimState = leadMotor.getSimState();

        leadConfig = new TalonFXConfiguration();
    
        leadConfig.Slot0.kG = constElevator.KG;
        leadConfig.Slot0.kS = constElevator.KS;
        leadConfig.Slot0.kV = constElevator.KV;
        // leadConfig.Slot0.KF = constElevator.KF;
        leadConfig.Slot0.kA = constElevator.KA;
        leadConfig.Slot0.kP = constElevator.KP;
        leadConfig.Slot0.kI = constElevator.KI;
        leadConfig.Slot0.kD = constElevator.KD;

        MotionMagicConfigs elevatorMotion = leadConfig.MotionMagic;        

        elevatorMotion.MotionMagicCruiseVelocity = constElevator.motionVelocity;
        elevatorMotion.MotionMagicAcceleration = constElevator.motionAcceleration;
        
        //elevatorMotion.MotionMagicExpo_kA = constElevator.EXPO_KA;
        //elevatorMotion.MotionMagicExpo_kV = constElevator.EXPO_KV;

        leadMotor.getConfigurator().apply(leadConfig);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));
        elevatorControl = new MotionMagicVoltage(0);
        
        // Simulation setup, Mechanism2d
        elevatorSim.setState(constElevator.minElevatorHeightMeters, 0); // start at bottom, stopped
        leadMotorSimState.setRawRotorPosition(0);
        leadMotorSimState.setRotorVelocity(0);
        elevatorBase = new Mechanism2d(3, 3);
        elevatorRoot = elevatorBase.getRoot("elevatorRoot", 2, 0);
        elevatorDraw = elevatorRoot.append(new MechanismLigament2d("elevator", 2, 90));
        this.elevatorMinimumLength = elevatorMinimumLength;
        SmartDashboard.putData("Elevator", elevatorBase);
        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            constElevator.elevatorGearing,
            constElevator.carriageMass,
            constElevator.elevatorDrumRadius,
            constElevator.minElevatorHeightMeters,
            constElevator.maxElevatorHeightMeters,
            true,
            0.5
        );

        //this.setElevatorHeight(0);


        
    }


    public Distance getElevatorHeight() {
        if (RobotBase.isReal() == false) {
            return Units.Meters.of(elevatorSim.getPositionMeters());

        } else{
            return Units.Meters.of(leadMotor.getPosition().getValueAsDouble()/constElevator.ROTATIONS_PER_METER);
        }
    }

    public void setElevatorHeight(double height) { // IN METERS
        //elevatorSim.setState(height, 0);
        leadMotor.setControl(elevatorControl.withPosition(height * constElevator.ROTATIONS_PER_METER));
        elevatorTarget = height;
    }



    @Override
    public void periodic() {
        elevatorDraw.setLength(getElevatorHeight().in(Meters));
        SmartDashboard.putNumber("Elevator Height", getElevatorHeight().in(Meters));
        SmartDashboard.putNumber("Elevator Target", elevatorTarget);
    
        
    }

    @Override
    public void simulationPeriodic() {
        // Simple physics simulation
        leadMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        followMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(leadMotorSimState.getMotorVoltage());
        elevatorSim.update(0.02);


        leadMotorSimState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);
        followMotorSimState.setRawRotorPosition(elevatorSim.getPositionMeters() * constElevator.ROTATIONS_PER_METER);

        leadMotorSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * constElevator.ROTATIONS_PER_METER);
        followMotorSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * constElevator.ROTATIONS_PER_METER);
        
    }
}
