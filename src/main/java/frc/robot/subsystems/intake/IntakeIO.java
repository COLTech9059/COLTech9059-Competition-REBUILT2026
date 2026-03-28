package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** */
public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionDegrees = 0.0;
    public double positionLeader = 0.0;
    public double positionFollower = 0.0;
    public AngularVelocity velocityLeader = null;
    public AngularVelocity velocityFollower = null;
    public Voltage voltageLeader = null;
    public Voltage voltageFollower = null;
    public double leaderAppliedTorque = 0.0;
    public double intakeSpeed = 0.0;
    public boolean isIntakeInLeft = false;
    public boolean isIntakeInRight = false;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPositionVoltage(double volts) {}

  public default void setSpeed(double baseSpeed) {}

  public default void setSpeed(double baseIntakeSpeed, double baseFeedSpeed) {}

  public default void runFeed(double speed) {}

  public default void setPosition(double speed, boolean out) {}

  public default void setPosition(double positionDegrees) {}

  public default void stopIntake() {}

  public default void stopPosition() {}

  public default void setPID(double kP, double kI, double kD, int motorNum) {}

  public default void setFF(double kS, double kV, double kA, int motorNum) {}

  public default void configureAll() {}
}
