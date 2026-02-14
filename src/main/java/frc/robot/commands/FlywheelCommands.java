package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class FlywheelCommands {

  // TODO: Test this command comp, it may not work; If not, create a separate command with the feed
  // system logic
  /**
   * Runs the flywheel at the given velocity setpoint; closed loop
   *
   * @param flywheel The Flywheel subsystem to use
   * @param velocityRPM The velocity setpoint in RPM
   */
  public static Command setVelocity(
      Flywheel flywheel, DoubleSupplier velocityRPM, double feedSpeed) {
    BooleanSupplier canRun =
        () ->
            (flywheel.getVelocityRPM() >= velocityRPM.getAsDouble() * 0.95
                && flywheel.getVelocityRPM() <= velocityRPM.getAsDouble() * 1.05);

    return Commands.run(() -> flywheel.runVelocity(velocityRPM.getAsDouble()), flywheel)
        .alongWith(
            Commands.run(() -> flywheel.runFeed(feedSpeed))
                .onlyWhile(canRun)
                .finallyDo(() -> flywheel.stopFeed()))
        .finallyDo(() -> flywheel.stop());
  }

  /**
   * Runs the flywheel at the given velocity setpoint; closed loop
   *
   * @param flywheel The Flywheel subsystem to use
   * @param velocityRadPerSec The velocity setpoint in radians per sec
   */
  public static Command setVelocityRad(Flywheel flywheel, double velocityRadPerSec) {
    return Commands.runOnce(
            () -> flywheel.runVelocity(Units.radiansToRotations(velocityRadPerSec)), flywheel)
        .finallyDo(() -> flywheel.stop());
  }

  /**
   * Runs the flywheel at the given voltage; open loop
   *
   * @param flywheel The Flywheel subsystem to use
   * @param volts The voltage to run the flywheel at
   */
  public static Command setVolts(Flywheel flywheel, double volts) {
    return Commands.runOnce(() -> flywheel.runVolts(volts), flywheel)
        .finallyDo(() -> flywheel.stop());
  }

  /**
   * Stops the flywheel
   *
   * @param flywheel The Flywheel subsystem to use
   */
  public static Command stop(Flywheel flywheel) {
    return Commands.runOnce(
        () -> {
          flywheel.stop();
          flywheel.stopFeed();
        },
        flywheel);
  }
}
