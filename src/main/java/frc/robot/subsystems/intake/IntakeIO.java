package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionDegrees = 0.0;
    public double intakeSpeed = 0.0;
    public boolean isIntakeInLeft = false;
    public boolean isIntakeInRight = false;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSpeed(double baseSpeed) {}

  public default void setSpeed(double baseIntakeSpeed, double baseFeedSpeed) {}

  public default void setPosition(double speed, boolean out) {}

  public default void stopIntake() {}

  public default void stopPosition() {}
}
