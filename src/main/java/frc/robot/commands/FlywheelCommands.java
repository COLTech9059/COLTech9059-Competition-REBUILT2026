package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

/**
 * @author DevAspen
 */
public class FlywheelCommands {

  // TODO: Test this command comp, it may not work; If not, create a separate command with the feed
  // system logic
  /**
   * Runs the flywheel at the given velocity setpoint; closed loop
   *
   * @param flywheel The Flywheel subsystem to use
   * @param velocityRPM The velocity setpoint in RPM
   * @param tolerance The tolerance of the setpoint in either direction, expressed as a decimal multiplier (i.e. 5% tolerance = 0.05)
   * @param feedSpeed The speed to run the feed system at
   */
  public static Command setVelocity(Flywheel flywheel, DoubleSupplier velocityRPM, double tolerance, double feedSpeed) {

    return Commands.run(() -> flywheel.runVelocity(velocityRPM.getAsDouble()), flywheel)
        .alongWith(
            Commands.run(() -> flywheel.runFeed(feedSpeed))
                .onlyWhile(() -> flywheel.isFlywheelUpToSpeed(velocityRPM.getAsDouble(), tolerance))
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
 * Runs the vertical feed system at the given speed
 * 
 * @param flywheel The Flywheel subsystem to use
 * @param speed The speed to run the feed system at
 */
  public static Command runFeed(Flywheel flywheel, double speed) {
    return Commands.runOnce(() -> flywheel.runFeed(speed));
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
