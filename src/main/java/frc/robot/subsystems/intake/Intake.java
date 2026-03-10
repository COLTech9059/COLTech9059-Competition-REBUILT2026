package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.extendedPositionDeadbandInRadians;
import static frc.robot.Constants.IntakeConstants.extendedPositionInRadians;

import edu.wpi.first.math.util.Units;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem, driven by several motors; Kraken, NEO, and hybrid compatibility
 *
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
  public void setPosition(double baseSpeed, boolean out) {
    io.setPosition(baseSpeed, out);

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

  public boolean isIntakeOut() {
    double currentAngle = Units.degreesToRadians(inputs.positionDegrees);
    return ((currentAngle >= (extendedPositionInRadians - extendedPositionDeadbandInRadians)));
  }

  public boolean isIntakeIn() {
    return (isIntakeInLeft() || isIntakeInRight());
  }

  /** Return true if the left intake sensor is active. */
  public boolean isIntakeInLeft() {
    return inputs.isIntakeInLeft;
  }

  /** Return true if the intake is 'in' */
  public boolean isIntakeInRight() {
    return inputs.isIntakeInRight;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
