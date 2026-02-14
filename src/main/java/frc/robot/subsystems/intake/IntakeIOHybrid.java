package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class IntakeIOHybrid implements IntakeIO {
  private SparkMax positionMotor =
      new SparkMax(INTAKE_POSITION.getDeviceNumber(), MotorType.kBrushless);
  private TalonFX intakeMotor = new TalonFX(INTAKE_ROLLER.getDeviceNumber());
  private SparkMax feedMotor = new SparkMax(INTAKE_FEED.getDeviceNumber(), MotorType.kBrushless);
  private RelativeEncoder encoder = positionMotor.getEncoder();
  private DigitalInput outLimit = new DigitalInput(INTAKE_OUT_LIMIT);
  private DigitalInput inLimit = new DigitalInput(INTAKE_IN_LIMIT);
  public final int[] powerPorts = {
    INTAKE_POSITION.getPowerPort(), INTAKE_ROLLER.getPowerPort(), INTAKE_FEED.getPowerPort()
  };

  private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();

  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public IntakeIOHybrid() {
    var positionConfig = new SparkMaxConfig();
    positionConfig
        .idleMode(IdleMode.kBrake)
        .inverted(kIntakePositionInverted)
        .smartCurrentLimit(kIntakeCurrentLimit)
        .voltageCompensation(kIntakeOptimalVoltage)
        .openLoopRampRate(kIntakeOpenLoopRampPeriod)
        .closedLoopRampRate(kIntakeClosedLoopRampPeriod);

    var feedConfig = positionConfig;
    feedConfig.idleMode(IdleMode.kCoast).inverted(kIntakeFeedInverted);

    SparkUtil.tryUntilOk(
        positionMotor,
        5,
        () ->
            positionMotor.configure(
                positionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        feedMotor,
        5,
        () ->
            feedMotor.configure(
                feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    intakeConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    if (kIntakeInverted)
      intakeConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else intakeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kIntakeOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kIntakeOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kIntakeOpenLoopRampPeriod;

    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kIntakeClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kIntakeClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kIntakeClosedLoopRampPeriod;

    // Apply the open & closed-loop ramp configuration for current smoothing
    intakeConfig.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);

    intakeMotor.getConfigurator().apply(intakeConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeCurrent);
    intakeMotor.optimizeBusUtilization();
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
          intakeCurrent.getValueAsDouble(),
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
