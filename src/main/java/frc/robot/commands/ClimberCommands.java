package frc.robot.commands;

import java.util.function.BooleanSupplier;

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

  /**
   * Toggles the climber up/down based on Supplier input
   * @param climber The climber subsystem to use
   * @param speed The speed to run the climber at
   * @param selector The selector to determine whether the climber extends or retracts; true = up, false = down
   */
  public static Command toggleClimber(Climber climber, double speed, BooleanSupplier selector) {
    if (selector.getAsBoolean()) return extendClimber(climber, speed);
    else return retractClimber(climber, speed);
  }

  public static Command stopClimber(Climber climber) {
    return Commands.runOnce(() -> climber.stop(), climber);
  }

}
