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
      new SparkMax(INTAKE_POSITION_MOTOR.getDeviceNumber(), MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR.getDeviceNumber(), MotorType.kBrushless);
  private RelativeEncoder encoder = positionMotor.getEncoder();
  private DigitalInput outLimit = new DigitalInput(INTAKE_OUT_LIMIT);
  private DigitalInput inLimit = new DigitalInput(INTAKE_IN_LIMIT);
  public final int[] powerPorts = {
    INTAKE_POSITION_MOTOR.getPowerPort(), INTAKE_MOTOR.getPowerPort()
  };

  public IntakeIOSpark() {
    var positionConfig = new SparkMaxConfig();
    positionConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(kIntakeCurrentLimit)
        .voltageCompensation(kIntakeOptimalVoltage)
        .openLoopRampRate(kIntakeOpenLoopRampPeriod)
        .closedLoopRampRate(kIntakeClosedLoopRampPeriod);

    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(kIntakeCurrentLimit)
        .voltageCompensation(kIntakeOptimalVoltage)
        .openLoopRampRate(kIntakeOpenLoopRampPeriod)
        .closedLoopRampRate(kIntakeClosedLoopRampPeriod);

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
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeSpeed = intakeMotor.get() * 100.0;
    inputs.positionDegrees = getIntakePos();
    inputs.isIntakeOut = isIntakeOut();
    inputs.currentAmps =
        new double[] {positionMotor.getOutputCurrent(), intakeMotor.getOutputCurrent()};

    // AdvantageKit logging
    Logger.recordOutput("Intake/IntakeSpeed", inputs.intakeSpeed);
    Logger.recordOutput("Intake/PositionDegrees", inputs.positionDegrees);
    Logger.recordOutput("Intake/IsIntakeOut", inputs.isIntakeOut);
    Logger.recordOutput("Intake/PositionCurrent", inputs.currentAmps[0]);
    Logger.recordOutput("Intake/IntakeCurrent", inputs.currentAmps[1]);
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
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
  }

  @Override
  public void stopPosition() {
    positionMotor.stopMotor();
  }

  @Override
  public double getIntakePos() {
    return Units.rotationsToDegrees(encoder.getPosition());
  }

  @Override
  public boolean isIntakeOut() {
    return outLimit.get();
  }
}
