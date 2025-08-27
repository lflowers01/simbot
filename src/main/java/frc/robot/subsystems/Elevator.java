package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;



import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class Elevator extends SubsystemBase{

    TalonFX leadMotor, followMotor;
    
    TalonFXConfiguration leadConfig, followConfig;
    final MotionMagicExpoVoltage elevatorControl;

    // Simulation variables
    Mechanism2d elevatorMech;
    MechanismRoot2d elevatorRoot;
    MechanismLigament2d elevator;
    double elevatorMinimumLength;

    double elevatorTarget;


    public Elevator(double elevatorMinimumLength) {

        leadMotor = new TalonFX(constElevator.LEAD_MOTOR_ID);
        followMotor = new TalonFX(constElevator.FOLLOW_MOTOR_ID);
        leadConfig = new TalonFXConfiguration();

        leadConfig.Slot0.kG = constElevator.KG;
        leadConfig.Slot0.kS = constElevator.KS;
        leadConfig.Slot0.kV = constElevator.KV;
        leadConfig.Slot0.kA = constElevator.KA;
        leadConfig.Slot0.kP = constElevator.KP;
        leadConfig.Slot0.kI = constElevator.KI;
        leadConfig.Slot0.kD = constElevator.KD;

        MotionMagicConfigs elevatorMotion = leadConfig.MotionMagic;

        elevatorMotion.MotionMagicCruiseVelocity = constElevator.motionVelocity;
        elevatorMotion.MotionMagicAcceleration = constElevator.motionAcceleration;
        elevatorMotion.MotionMagicExpo_kA = constElevator.EXPO_KA;
        elevatorMotion.MotionMagicExpo_kV = constElevator.EXPO_KV;

        leadMotor.getConfigurator().apply(leadConfig);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));
        elevatorControl = new MotionMagicExpoVoltage(0);
        
        // Simulation setup, Mechanism2d

        elevatorMech = new Mechanism2d(3, 3);
        elevatorRoot = elevatorMech.getRoot("elevatorRoot", 2, 0);
        elevator = elevatorRoot.append(new MechanismLigament2d("elevator", 2, 90));
        this.elevatorMinimumLength = elevatorMinimumLength;
        SmartDashboard.putData("Elevator", elevatorMech);

        this.setElevatorHeight(0);
        
    }


    public Distance getElevatorHeight() {
        return Units.Inches.of(leadMotor.getPosition().getValueAsDouble());
    }

    public void setElevatorHeight(double height) {
        leadMotor.setControl(elevatorControl.withPosition(height));
        elevatorTarget = height;
    }

    @Override
    public void periodic() {
        elevator.setLength(elevatorMinimumLength + getElevatorHeight().in(Meters));
        SmartDashboard.putNumber("Elevator Height", getElevatorHeight().in(Meters));
        SmartDashboard.putNumber("Elevator Target", elevatorTarget);
        
        
    }
}
