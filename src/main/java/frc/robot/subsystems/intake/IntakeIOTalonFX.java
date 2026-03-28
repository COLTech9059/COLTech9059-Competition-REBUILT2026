package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX positionMotor = new TalonFX(INTAKE_POSITION.getDeviceNumber());
  private TalonFX positionMotorFollower = new TalonFX(INTAKE_POSITION_FOLLOWER.getDeviceNumber());
  private TalonFX intakeMotor = new TalonFX(INTAKE_ROLLER.getDeviceNumber());
  private TalonFX feedMotor = new TalonFX(INTAKE_FEED.getDeviceNumber());
  private DigitalInput inLimitLeft = new DigitalInput(INTAKE_IN_LEFT_LIMIT);
  private DigitalInput inLimitRight = new DigitalInput(INTAKE_IN_RIGHT_LIMIT);
  public final int[] powerPorts = {
    INTAKE_POSITION.getPowerPort(), INTAKE_ROLLER.getPowerPort(), INTAKE_FEED.getPowerPort()
  };

  private final StatusSignal<Angle> positionLeader = positionMotor.getPosition();
  private final StatusSignal<Angle> positionFollower = positionMotorFollower.getPosition();

  private final StatusSignal<Current> positionCurrent = positionMotor.getSupplyCurrent();
  private final StatusSignal<Current> positionFollowerCurrent =
      positionMotorFollower.getSupplyCurrent();
  private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Current> feedCurrent = feedMotor.getSupplyCurrent();
  private final StatusSignal<Current> leaderAppliedTorque = positionMotor.getTorqueCurrent();

  private final TalonFXConfiguration positionConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration positionFollowerConfig;
  private final TalonFXConfiguration intakeConfig;
  private final TalonFXConfiguration feedConfig;

  public IntakeIOTalonFX() {
    positionConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    positionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    positionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (kIntakePositionInverted)
      positionConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
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

    intakeConfig = positionConfig.clone();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    if (kIntakeInverted)
      intakeConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else intakeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    feedConfig = positionConfig.clone();
    feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    if (kIntakeFeedInverted)
      feedConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else feedConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    positionFollowerConfig = positionConfig.clone();

    positionMotor.getConfigurator().apply(positionConfig);
    positionMotorFollower.getConfigurator().apply(positionFollowerConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
    feedMotor.getConfigurator().apply(feedConfig);

    leaderAppliedTorque.setUpdateFrequency(200);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionLeader,
        positionFollower,
        positionCurrent,
        intakeCurrent,
        feedCurrent,
        positionFollowerCurrent);
    positionMotor.optimizeBusUtilization();
    positionMotorFollower.setControl(
        new Follower(positionMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionLeader,
        positionFollower,
        positionCurrent,
        intakeCurrent,
        feedCurrent,
        leaderAppliedTorque,
        positionFollowerCurrent);
    inputs.intakeSpeed = intakeMotor.get() * 100.0;
    inputs.positionDegrees = getRotationsInDegrees();
    inputs.positionLeader =
        Units.rotationsToDegrees(positionLeader.getValueAsDouble()) / kIntakeGearRatio;
    inputs.positionFollower =
        Units.rotationsToDegrees(positionFollower.getValueAsDouble()) / kIntakeGearRatio;
    inputs.leaderAppliedTorque = leaderAppliedTorque.getValueAsDouble();
    inputs.isIntakeInLeft = inLimitLeft.get();
    inputs.isIntakeInRight = inLimitRight.get();
    inputs.currentAmps =
        new double[] {
          positionCurrent.getValueAsDouble(),
          intakeCurrent.getValueAsDouble(),
          feedCurrent.getValueAsDouble(),
          positionFollowerCurrent.getValueAsDouble()
        };

    // Reset position encoder.
    if (inLimitLeft.get() || inLimitRight.get()) setEncoderRotations(0);
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
    feedMotor.setVoltage(volts);
  }

  @Override
  public void setPositionVoltage(double volts) {
    positionMotor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
    // feedMotor.set(speed);
  }

  @Override
  public void setSpeed(double baseIntakeSpeed, double baseFeedSpeed) {
    intakeMotor.set(baseIntakeSpeed);
    feedMotor.set(baseFeedSpeed);
  }

  @Override
  public void runFeed(double speed) {
    feedMotor.set(speed);
  }

  @Override
  public void setPosition(double baseSpeed, boolean out) {
    double currentAngle = getRotationsInRadians();
    baseSpeed = Math.abs(baseSpeed);
    double appliedSpeed = baseSpeed;

    if (out) {
      double angleDifference = extendedPositionInRadians - currentAngle;
      if (angleDifference < extendedPositionDeadbandInRadians) appliedSpeed = 0;
      // else appliedSpeed = Math.max(baseSpeed, 0.5 * Math.sin(angleDifference));
      else appliedSpeed = -baseSpeed;
    } else {
      if (!(inLimitLeft.get() || inLimitRight.get()))
        // appliedSpeed = -Math.max(baseSpeed, 0.5 * Math.sin(currentAngle));
        appliedSpeed = baseSpeed;
      else appliedSpeed = 0;
    }
    positionMotor.set(appliedSpeed);
  }

  @Override
  public void setPosition(double positionDegrees) {
    positionMotor.setControl(
        new PositionTorqueCurrentFOC(Units.degreesToRotations(positionDegrees)));
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

  // Encoder Positioning
  private double getAverageRotations() {
    return (positionLeader.getValueAsDouble() + positionFollower.getValueAsDouble()) / 2;
  }

  private void setEncoderRotations(double rotations) {
    positionMotor.setPosition(rotations);
    positionMotorFollower.setPosition(rotations);
  }

  private double getRotationsInRadians() {
    double motorRotationInRadians = Units.rotationsToRadians(getAverageRotations());
    return motorRotationInRadians / kIntakePositionGearRatio;
  }

  private double getRotationsInDegrees() {
    double motorRotationInDegrees = Units.rotationsToDegrees(getAverageRotations());
    return motorRotationInDegrees / kIntakeGearRatio;
  }

  @Override
  public void setPID(double kP, double kI, double kD, int motorNum) {
    switch (motorNum) {
      case 1:
        positionConfig.Slot0.kP = kP;
        positionConfig.Slot0.kI = kI;
        positionConfig.Slot0.kD = kD;
        break;
      case 2:
        positionFollowerConfig.Slot0.kP = kP;
        positionFollowerConfig.Slot0.kI = kI;
        positionFollowerConfig.Slot0.kD = kD;
        break;
    }
  }

  @Override
  public void setFF(double kS, double kV, double kA, int motorNum) {
    switch (motorNum) {
      case 1:
        positionConfig.Slot0.kS = kS;
        positionConfig.Slot0.kV = kV;
        positionConfig.Slot0.kA = kA;
        break;
      case 2:
        positionFollowerConfig.Slot0.kS = kS;
        positionFollowerConfig.Slot0.kV = kV;
        positionFollowerConfig.Slot0.kA = kA;
        break;
    }
  }

  @Override
  public void configureAll() {
    PhoenixUtil.tryUntilOk(5, () -> positionMotor.getConfigurator().apply(positionConfig));
    PhoenixUtil.tryUntilOk(
        5, () -> positionMotorFollower.getConfigurator().apply(positionFollowerConfig));
  }
}
