package frc.robot.subsystems.intake;

import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends RBSISubsystem {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setPosition(double speed, boolean out) {
    io.setPosition(speed, out);

    Logger.recordOutput("Intake/setpointPosition", out);
  }

  /** Stops the intake motor */
  public void stopIntake() {
    io.stopIntake();
  }

  /** Stops the intake position motor */
  public void stopPosition() {
    io.stopPosition();
  }

  @AutoLogOutput(key = "Mechanism/Intake")
  public double getIntakePos() {
    return io.getIntakePos();
  }

  @AutoLogOutput(key = "Mechanism/Intake")
  public boolean isIntakeOut() {
    return io.isIntakeOut();
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
