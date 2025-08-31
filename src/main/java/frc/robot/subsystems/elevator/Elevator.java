package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
    System.out.println("Elevator constructed with IO: " + io.getClass().getSimpleName());
    System.out.println("Elevator rotations per meter:" + constElevator.ROTATIONS_PER_METER);
  }

  @Override
  public void periodic() {
    // System.out.println("Elevator periodic");
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setHeight(double meters) {
    io.setHeight(meters);
  }

  
}
