package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.RBSISubsystem;

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

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setPosition(double speed, boolean up) {
    io.setPosition(speed, up);
  }

  public void stop() {
    io.stop();
  }

  public double getClimberPos() {
    return inputs.positionInches;
  }

  public boolean isClimberUp() {
    return inputs.isClimberUp;
  }

  public boolean isClimberDown() {
    return inputs.isClimberDown;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
