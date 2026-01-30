package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;

public class FlywheelCommands {
  
  /**
   * Runs the flywheel at the given velocity setpoint; closed loop
   * @param flywheel The Flywheel subsystem to use
   * @param velocityRPM The velocity setpoint in RPM
   */
  public static Command setVelocity(Flywheel flywheel, double velocityRPM) {
    return Commands.runOnce(() -> flywheel.runVelocity(velocityRPM), flywheel);
  }

  /**
   * Runs the flywheel at the given velocity setpoint; closed loop
   * @param flywheel The Flywheel subsystem to use
   * @param velocityRadPerSec The velocity setpoint in radians per sec
   */
  public static Command setVelocityRad(Flywheel flywheel, double velocityRadPerSec) {
    return Commands.runOnce(() -> flywheel.runVelocity(Units.radiansToRotations(velocityRadPerSec)), flywheel);
  }

  /**
   * Runs the flywheel at the given voltage; open loop
   * @param flywheel The Flywheel subsystem to use
   * @param volts The voltage to run the flywheel at
   */
  public static Command setVolts(Flywheel flywheel, double volts) {
    return Commands.runOnce(() -> flywheel.runVolts(volts), flywheel);
  }

  /**
   * Stops the flywheel
   * @param flywheel The Flywheel subsystem to use
   */
  public static Command stop(Flywheel flywheel) {
    return Commands.runOnce(() -> flywheel.stop(), flywheel);
  }
}
