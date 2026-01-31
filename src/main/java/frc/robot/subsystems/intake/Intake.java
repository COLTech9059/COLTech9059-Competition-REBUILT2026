package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Intake", inputs);
  }
  
  public void runVolts(double volts) {
    // io.setVoltage(volts);
  }

  public void runSpeed(double speed) {
    // io.setSpeed(speed);
  }

  public void setPosition(double speed, boolean out) {
    // io.setIntakePosition(speed, out);

    // Logger.recordOutput("Intake/setpointPosition", position);
  }

  /**
   * Stops the intake motor
   */
  public void stopIntake() {
    // io.stopIntake();
  }

  /**
   * Stops the intake position motor
   */
  public void stopPosition() {
    // io.stopPosition();
  }

  @AutoLogOutput(key = "Mechanism/Intake")
  public double getIntakePos() {
    return 0; // io.getIntakePos();
  }

  @AutoLogOutput(key = "Mechanism/Intake")
  public boolean isIntakeOut() {
    return false; // io.isIntakeOut();
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
