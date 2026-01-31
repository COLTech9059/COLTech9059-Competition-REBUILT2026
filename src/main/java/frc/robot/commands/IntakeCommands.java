package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  
  public static Command extendIntake(Intake intake, double speed) {
    BooleanSupplier canRun = () -> !intake.isIntakeOut();
    return Commands.run(() -> intake.setPosition(speed, true), intake)
      .onlyIf(canRun)
      .until(() -> intake.isIntakeOut())
      .finallyDo(() -> intake.stopPosition());
  }

  public static Command retractIntake(Intake intake, double speed) {
    BooleanSupplier canRun = () -> !intake.isIntakeIn();
    return Commands.run(() -> intake.setPosition(speed, false), intake)
      .onlyIf(canRun)
      .until(() -> intake.isIntakeIn())
      .finallyDo(() -> intake.stopPosition());
  }

  public static Command runIntakeSpeed(Intake intake, double speed) {
    return Commands.startEnd(() -> intake.runSpeed(speed), () -> intake.stopIntake(), intake);
  }

  public static Command runIntakeVolts(Intake intake, double volts) {
    return Commands.startEnd(() -> intake.runVolts(volts), () -> intake.stopIntake(),intake);
  }

  public static Command stopIntake(Intake intake) {
    return Commands.runOnce(() -> intake.stopIntake(), intake);
  }

  public static Command intakeSequence(Intake intake, double positionSpeed, double intakeSpeed) {
    return Commands.sequence(
      extendIntake(intake, positionSpeed),
      runIntakeSpeed(intake, intakeSpeed)
    );
  }
}
