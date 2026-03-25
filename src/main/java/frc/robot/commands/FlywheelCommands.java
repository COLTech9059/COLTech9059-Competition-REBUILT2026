package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

/**
 * @author DevAspen
 */
public class FlywheelCommands {

  /**
   * Runs the flywheel at the given velocity setpoint and automatically manages feeding; closed loop
   *
   * @param flywheel The Flywheel subsystem to use
   * @param velocityRPM The velocity setpoint in RPM
   * @param tolerance The tolerance of the setpoint in either direction, expressed as a decimal
   *     multiplier (i.e. 5% tolerance = 0.05)
   * @param feedSpeed The speed to run the uptake system at
   * @param indexSpeed The speed to run the indexing system at
   * @param intakeSpeed The speed to run the intake rollers at
   */
  public static Command setVelocityAutomatic(
      Flywheel flywheel,
      Intake intake,
      DoubleSupplier velocityRPM,
      double tolerance,
      double feedSpeed,
      double indexSpeed,
      double intakeSpeed) {
    return Commands.run(() -> flywheel.runVelocity(velocityRPM.getAsDouble()), flywheel)
        .alongWith(
            Commands.run(
                    () -> {
                      flywheel.runFeed(feedSpeed);
                      intake.setSpeed(intakeSpeed, indexSpeed);
                    })
                .onlyWhile(() -> flywheel.isFlywheelUpToSpeed(velocityRPM.getAsDouble(), tolerance))
                .finallyDo(
                    () -> {
                      flywheel.stopFeed();
                      intake.stopIntake();
                    }))
        .finallyDo(
            () -> {
              flywheel.stop();
              flywheel.stopFeed();
              intake.stopIntake();
            });
  }

  public static Command setVelocity(Flywheel flywheel, DoubleSupplier velocityRPM) {
    return Commands.run(() -> flywheel.runVelocity(velocityRPM.getAsDouble()), flywheel)
        .finallyDo(() -> flywheel.stop());
  }

  public static Command setVelocity(
      Flywheel flywheel, DoubleSupplier velocityRPM, double unclogTime, double unclogSpeed) {
    return Commands.sequence(
            Commands.runOnce(() -> flywheel.runFeed(unclogSpeed)),
            Commands.waitSeconds(unclogTime),
            Commands.runOnce(() -> flywheel.stopFeed()),
            Commands.runOnce(() -> flywheel.runVelocity(velocityRPM.getAsDouble()), flywheel))
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
            () -> flywheel.runVelocity(Units.radiansToRotations(velocityRadPerSec) * 60), flywheel)
        .finallyDo(() -> flywheel.stop());
  }

  /**
   * Continuous command sequence to pulse the whole uptake system at the given speeds and interval
   * <b>(Does not stop these motors on interrupt!)</b>
   *
   * @param flywheel The Flywheel subsystem to use
   * @param intake The Intake subsystem to use
   * @param uptakeSpeed The speed to run the uptake/flywheel feed system at
   * @param indexSpeed The speed to run the indexing system at
   * @param intakeSpeed The speed to run the intake rollers at
   * @param pulseInterval The time between pulses (seconds)
   * @param pulseDuration The time for which each pulse runs (seconds)
   */
  public static Command pulseUptake(
      Flywheel flywheel,
      Intake intake,
      double uptakeSpeed,
      double indexSpeed,
      double intakeSpeed,
      double pulseInterval,
      double pulseDuration) {
    return Commands.sequence(
            IntakeCommands.runIntakeSpeed(intake, intakeSpeed, indexSpeed),
            runFeed(flywheel, uptakeSpeed),
            Commands.waitSeconds(pulseDuration),
            IntakeCommands.stopIntake(intake),
            stop(flywheel),
            Commands.waitSeconds(pulseInterval))
        .repeatedly();
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
    return Commands.runOnce(
        () -> {
          flywheel.runFeed(speed);
          flywheel.startTimer();
        });
  }

  public static Command setSpeed(Flywheel flywheel) {
    return Commands.runOnce(() -> flywheel.setSpeed());
  }

  public static Command setSpeed(Flywheel flywheel, double speed) {
    return Commands.runOnce(() -> flywheel.setSpeed(speed));
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
          flywheel.stopTimer();
        },
        flywheel);
  }

  public static Command stopFeed(Flywheel flywheel) {
    return Commands.runOnce(() -> flywheel.stopFeed());
  }
}
