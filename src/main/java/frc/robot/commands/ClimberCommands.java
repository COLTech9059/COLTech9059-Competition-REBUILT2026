package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommands {
  
  public static Command extendClimber(Climber climber, double speed) {
    return Commands.run(() -> climber.setPosition(speed, true), climber)
      .until(climber::isClimberUp)
      .finallyDo(() -> climber.stop());
  }

  public static Command retractClimber(Climber climber, double speed) {
    return Commands.run(() -> climber.setPosition(-speed, false), climber)
      .until(climber::isClimberDown)
      .finallyDo(() -> climber.stop());
  }

  public static Command stopClimber(Climber climber) {
    return Commands.runOnce(() -> climber.stop(), climber);
  }

}
