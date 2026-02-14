package frc.robot.subsystems.climber;

import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem, driven by a single motor; Kraken and NEO compatibility
 *
 * @author DevAspen
 */
public class Climber extends RBSISubsystem {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /** Set the speed of the climber */
  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Set the climber to the up or down position */
  public void setPosition(double speed, boolean up) {
    io.setPosition(speed, up);
  }

  /** Stop the climber */
  public void stop() {
    io.stop();
  }

  /** Get the position of the climber in inches above its base */
  public double getClimberPos() {
    return inputs.positionInches;
  }

  /** Return true if the climber is in the 'up' position */
  public boolean isClimberUp() {
    return inputs.isClimberUp;
  }

  /** Return true if the climber is in the 'down' position */
  public boolean isClimberDown() {
    return inputs.isClimberDown;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
