package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionDegrees = 0.0;
    public double intakeSpeed = 0.0;
    public boolean isIntakeOut = false;
    public boolean isIntakeIn = false;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSpeed(double speed) {}

  public default void setPosition(double speed, boolean out) {}

  public default void stopIntake() {}

  public default void stopPosition() {}
}
