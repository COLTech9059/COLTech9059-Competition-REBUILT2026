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
  public static Command extendIntake(Intake intake, double speed) {
    return Commands.run(() -> intake.setPosition(speed, true), intake)
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
  public static Command retractIntake(Intake intake, double speed) {
    return Commands.run(() -> intake.setPosition(speed, false), intake)
        .unless(intake::isIntakeIn)
        .until(intake::isIntakeIn)
        .finallyDo(() -> intake.stopPosition());
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
}
