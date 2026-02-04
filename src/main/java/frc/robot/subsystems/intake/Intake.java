package frc.robot.subsystems.intake;

import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem, driven by several motors; Kraken, NEO, and hybrid compatibility
 * @author DevAspen
 */
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

  /** Run the intake rollers at the specified voltage */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run the intake rollers at the specified speed */
  public void runSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Set the position of the intake, either extended or retracted */
  public void setPosition(double speed, boolean out) {
    io.setPosition(speed, out);

    Logger.recordOutput("Intake/setpointPosition", out);
  }

  /** Stop the intake rollers */
  public void stopIntake() {
    io.stopIntake();
  }

  /** Stop the intake position motor */
  public void stopPosition() {
    io.stopPosition();
  }

  /** Get the intake position in degrees */
  public double getIntakePos() {
    return inputs.positionDegrees;
  }

  /** Return true if the intake is 'out' */
  public boolean isIntakeOut() {
    return inputs.isIntakeOut;
  }

  /** Return true if the intake is 'in' */
  public boolean isIntakeIn() {
    return inputs.isIntakeIn;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
