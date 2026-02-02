package frc.robot.subsystems.climber;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.SparkUtil;

public class ClimberIOSpark implements ClimberIO {

  private SparkMax climbMotor = new SparkMax(CLIMBER_MOTOR.getDeviceNumber(), MotorType.kBrushless);
  private RelativeEncoder encoder = climbMotor.getEncoder();
  private DigitalInput upLimit = new DigitalInput(CLIMBER_UP_LIMIT);
  private DigitalInput downLimit = new DigitalInput(CLIMBER_DOWN_LIMIT);
  public final int[] powerPorts = {
    CLIMBER_MOTOR.getPowerPort()
  };

  public ClimberIOSpark() {
    SparkMaxConfig config = new SparkMaxConfig();
    switch (kClimberIdleMode) {
      case BRAKE:
        config.idleMode(IdleMode.kBrake);
        break;
      case COAST:
        config.idleMode(IdleMode.kCoast);
        break;
    };

    config
    .inverted(kClimberInverted)
    .smartCurrentLimit(kClimberCurrentLimit)
    .voltageCompensation(kClimberOptimalVoltage)
    .openLoopRampRate(kClimberOpenLoopRampPeriod)
    .closedLoopRampRate(kClimberClosedLoopRampPeriod);

    SparkUtil.tryUntilOk(
        climbMotor,
        5,
        () ->
            climbMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberSpeed = climbMotor.get() * 100.0;
    inputs.positionInches = encoder.getPosition() / kClimberRotationsToInchesRatio;
    inputs.isClimberUp = upLimit.get();
    inputs.isClimberDown = downLimit.get();
    inputs.currentAmps = new double[] {climbMotor.getOutputCurrent()};

    // AdvantageKit logging
    Logger.recordOutput("Climber/ClimberSpeed", inputs.climberSpeed);
    Logger.recordOutput("Climber/PositionInches", inputs.positionInches);
    Logger.recordOutput("Climber/IsClimberUp", inputs.isClimberUp);
    Logger.recordOutput("Climber/IsClimberDown", inputs.isClimberDown);
    Logger.recordOutput("Climber/ClimberCurrent", inputs.currentAmps);
  }

  @Override
  public void setSpeed(double speed) {
    climbMotor.set(speed);
  }

  @Override
  public void setPosition(double speed, boolean up) {
    speed = Math.abs(speed);
    
    if (up) {
      if (upLimit.get()) stop();
      else setSpeed(speed);
    }
    else {
      if (downLimit.get()) stop();
      else setSpeed(-speed);
    }
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
  }
}
