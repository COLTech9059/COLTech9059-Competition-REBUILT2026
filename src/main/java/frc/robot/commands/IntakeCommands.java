package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

  /**
   * Move the intake to the extended position at the specified speed
   *
   * @param intake The intake subsystem to use
   * @param speed The speed to run the motor at
   */
  public static Command extendIntake(Intake intake, double baseSpeed) {
    return Commands.run(() -> intake.setPosition(baseSpeed, true), intake)
        .unless(intake::isIntakeOut)
        .until(intake::isIntakeOut)
        .finallyDo(() -> intake.stopPosition());
  }

  /**
   * Move the intake to the retracted position at the specified speed
   *
   * @param intake The intake subsystem to use
   * @param speed The speed to run the motor at
   */
  public static Command retractIntake(Intake intake, double baseSpeed) {
    return Commands.run(() -> intake.setPosition(baseSpeed, false), intake)
        .unless(intake::isIntakeIn)
        .until(intake::isIntakeIn)
        .finallyDo(
            () -> {
              intake.stopPosition();
              intake.stopIntake();
            });
  }

  /**
   * Run the intke rollers at the specified speed
   *
   * @param intake The intake subsystem to use
   * @param speed The speed to run the rollers at
   */
  public static Command runIntakeSpeed(Intake intake, double speed) {
    return Commands.startEnd(() -> intake.runSpeed(speed), () -> intake.stopIntake(), intake);
  }

  public static Command runIntakeSpeed(Intake intake, double intakeSpeed, double feedSpeed) {
    return Commands.startEnd(
        () -> intake.setSpeed(intakeSpeed, feedSpeed), () -> intake.stopIntake());
  }

  /**
   * Run the intake rollers at the specified votlage
   *
   * @param intake The intake subsystem to use
   * @param volts The voltage to run the rollers at
   */
  public static Command runIntakeVolts(Intake intake, double volts) {
    return Commands.startEnd(() -> intake.runVolts(volts), () -> intake.stopIntake(), intake);
  }

  /**
   * Stop the intake rollers
   *
   * @param intake The intake subsystem to use
   */
  public static Command stopIntake(Intake intake) {
    return Commands.runOnce(() -> intake.stopIntake(), intake);
  }

  /**
   * Extend the intake, if applicable, then run the intake rollers
   *
   * @param intake The intake subsystem to use
   * @param positionSpeed The speed to move the intake at
   * @param intakeSpeed The speed to run the rollers at
   */
  public static Command intakeSequence(Intake intake, double positionSpeed, double intakeSpeed) {
    return Commands.sequence(
        extendIntake(intake, positionSpeed), runIntakeSpeed(intake, intakeSpeed));
  }

  /**
   * Set the intake to the given position
   * @param intake The intake subsystem to use
   * @param positionSetpointDegrees The angle setpoint to drive the intake to (degrees)
   */
  public static Command setIntakePosition(Intake intake, double positionSetpointDegrees) {
    return Commands.runOnce(() -> intake.setPosition(positionSetpointDegrees));
  }

  /**
   * Set the intake to the given position and oscillate
   * @param intake The intake subsystem to use
   * @param initialPositionDegrees The initial position to drive to before oscillation starts (degrees)
   * @param oscillatePositionDegrees The secondary position for oscillation (degrees)
   * @param delayTimeSeconds The delay between driving to the setpoints (seconds)
   */
  public static Command oscillateIntakePosition(Intake intake, double initialPositionDegrees, double oscillatePositionDegrees, double delayTimeSeconds, double intakeSetpointRPM, double feedSpeed) {
    return Commands.sequence(
      setIntakePosition(intake, initialPositionDegrees),
      Commands.waitSeconds(delayTimeSeconds),
      setIntakePosition(intake, oscillatePositionDegrees),
      Commands.waitSeconds(delayTimeSeconds))
      .repeatedly()
      .alongWith(setIntakeVelocity(intake, intakeSetpointRPM, feedSpeed))
      .finallyDo(() -> intake.stopPosition());
  }

  /**
   * Set the intake rollers to the given velocity setpoint and run the feed at the given duty cycle
   * @param intake The intake subsystem to use
   * @param velocityRPM The velocity to run the intake rollers at (rotations per minute)
   * @param feedSpeed The speed to run the indexing system at (duty cycle %)
   */
  public static Command setIntakeVelocity(Intake intake, double velocityRPM, double feedSpeed) {
    return Commands.runOnce(() -> intake.setIntakeVelocity(velocityRPM, feedSpeed));
  }
}
