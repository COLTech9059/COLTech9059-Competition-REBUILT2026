package frc.robot.subsystems.climber;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * Template Input/Output interface to act as a medium between the various "hardware" files and the
 * high-level subsystem file which the rest of the program interacts with
 *
 * @author DevAspen
 */
public interface ClimberIO extends RBSIIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double positionInches = 0.0;
    public double climberSpeed = 0.0;
    public boolean isClimberUp = false;
    public boolean isClimberDown = false;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void setPosition(double speed, boolean up) {}

  public default void stop() {}
}
