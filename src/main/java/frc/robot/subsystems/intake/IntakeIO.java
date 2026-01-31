package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionDegrees = 0.0;
    public double intakeSpeed = 0.0;
    public boolean isIntakeOut = false;
    public double[] currentAmps = new double[] {};
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setVoltage(double volts);

  public void setSpeed(double speed);

  public void setPosition(double speed, boolean out);

  public void stopIntake();

  public void stopPosition();

  public double getIntakePos();

  public boolean isIntakeOut();

  public boolean isIntakeIn();
}
