package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSpark implements IntakeIO {

  private SparkMax positionMotor =
      new SparkMax(INTAKE_POSITION.getDeviceNumber(), MotorType.kBrushless);
  private SparkMax intakeMotor =
      new SparkMax(INTAKE_ROLLER.getDeviceNumber(), MotorType.kBrushless);
  private SparkMax feedMotor = new SparkMax(INTAKE_FEED.getDeviceNumber(), MotorType.kBrushless);
  private RelativeEncoder encoder = positionMotor.getEncoder();
  private DigitalInput outLimit = new DigitalInput(INTAKE_OUT_LIMIT);
  private DigitalInput inLimit = new DigitalInput(INTAKE_IN_LIMIT);
  public final int[] powerPorts = {
    INTAKE_POSITION.getPowerPort(), INTAKE_ROLLER.getPowerPort(), INTAKE_FEED.getPowerPort()
  };

  public IntakeIOSpark() {
    var positionConfig = new SparkMaxConfig();
    positionConfig
        .idleMode(IdleMode.kBrake)
        .inverted(kIntakePositionInverted)
        .smartCurrentLimit(kIntakeCurrentLimit)
        .voltageCompensation(kIntakeOptimalVoltage)
        .openLoopRampRate(kIntakeOpenLoopRampPeriod)
        .closedLoopRampRate(kIntakeClosedLoopRampPeriod);

    var intakeConfig = positionConfig;
    intakeConfig.idleMode(IdleMode.kCoast).inverted(kIntakeInverted);

    var feedConfig = positionConfig;
    feedConfig.idleMode(IdleMode.kCoast).inverted(kIntakeFeedInverted);

    SparkUtil.tryUntilOk(
        positionMotor,
        5,
        () ->
            positionMotor.configure(
                positionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        feedMotor,
        5,
        () ->
            feedMotor.configure(
                feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeSpeed = intakeMotor.get() * 100.0;
    inputs.positionDegrees = Units.rotationsToDegrees(encoder.getPosition()) / kIntakeGearRatio;
    inputs.isIntakeOut = outLimit.get();
    inputs.isIntakeIn = inLimit.get();
    inputs.currentAmps =
        new double[] {
          positionMotor.getOutputCurrent(),
          intakeMotor.getOutputCurrent(),
          feedMotor.getOutputCurrent()
        };

    // AdvantageKit logging
    Logger.recordOutput("Intake/IntakeSpeed", inputs.intakeSpeed);
    Logger.recordOutput("Intake/PositionDegrees", inputs.positionDegrees);
    Logger.recordOutput("Intake/IsIntakeOut", inputs.isIntakeOut);
    Logger.recordOutput("Intake/IsIntakeIn", inputs.isIntakeIn);
    Logger.recordOutput("Intake/PositionCurrent", inputs.currentAmps[0]);
    Logger.recordOutput("Intake/IntakeCurrent", inputs.currentAmps[1]);
    Logger.recordOutput("Intake/FeedCurrent", inputs.currentAmps[2]);
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
    feedMotor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
    feedMotor.set(speed);
  }

  @Override
  public void setPosition(double speed, boolean out) {
    speed = Math.abs(speed);

    if (out) {
      if (outLimit.get()) stopPosition();
      else setSpeed(speed);
    } else {
      if (inLimit.get()) stopPosition();
      else setSpeed(-speed);
    }
  }

  @Override
  public void stopIntake() {
    intakeMotor.stopMotor();
    feedMotor.stopMotor();
  }

  @Override
  public void stopPosition() {
    positionMotor.stopMotor();
  }
}
