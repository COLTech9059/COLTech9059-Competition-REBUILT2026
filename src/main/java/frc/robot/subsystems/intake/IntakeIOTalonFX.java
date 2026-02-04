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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX positionMotor = new TalonFX(INTAKE_POSITION.getDeviceNumber());
  private TalonFX intakeMotor = new TalonFX(INTAKE_ROLLER.getDeviceNumber());
  private TalonFX feedMotor = new TalonFX(INTAKE_FEED.getDeviceNumber());
  private DigitalInput outLimit = new DigitalInput(INTAKE_OUT_LIMIT);
  private DigitalInput inLimit = new DigitalInput(INTAKE_IN_LIMIT);
  public final int[] powerPorts = {
    INTAKE_POSITION.getPowerPort(), INTAKE_ROLLER.getPowerPort(), INTAKE_FEED.getPowerPort()
  };

  private final StatusSignal<Angle> position = positionMotor.getPosition();
  private final StatusSignal<Current> positionCurrent = positionMotor.getSupplyCurrent();
  private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();

  private final TalonFXConfiguration positionConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration intakeConfig;
  private final TalonFXConfiguration feedConfig;

  public IntakeIOTalonFX() {
    positionConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    positionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    positionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (kIntakePositionInverted) positionConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else positionConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

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
    positionConfig.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);

    intakeConfig = positionConfig;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    if (kIntakeInverted) intakeConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else intakeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    feedConfig = positionConfig;
    feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    if (kIntakeFeedInverted) feedConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else feedConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    positionMotor.getConfigurator().apply(positionConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
    feedMotor.getConfigurator().apply(feedConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, positionCurrent, intakeCurrent);
    positionMotor.optimizeBusUtilization();
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, positionCurrent, intakeCurrent);
    inputs.intakeSpeed = intakeMotor.get() * 100.0;
    inputs.positionDegrees =
        Units.rotationsToDegrees(position.getValueAsDouble()) / kIntakePositionGearRatio;
    inputs.isIntakeOut = outLimit.get();
    inputs.isIntakeIn = inLimit.get();
    inputs.currentAmps =
        new double[] {positionCurrent.getValueAsDouble(), intakeCurrent.getValueAsDouble()};
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
      else positionMotor.set(speed);
    } else {
      if (inLimit.get()) stopPosition();
      else positionMotor.set(-speed);
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
