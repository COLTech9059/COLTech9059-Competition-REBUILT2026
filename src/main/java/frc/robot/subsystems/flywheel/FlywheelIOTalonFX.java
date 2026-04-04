// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel;

import static frc.robot.Constants.FlywheelConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

  // Define the leader / follower motors from the Ports section of RobotContainer
  private final TalonFX leader =
      new TalonFX(FLYWHEEL_LEADER.getDeviceNumber(), FLYWHEEL_LEADER.getCANBus());
  private final TalonFX follower =
      new TalonFX(FLYWHEEL_FOLLOWER.getDeviceNumber(), FLYWHEEL_FOLLOWER.getCANBus());
  private final TalonFX secondaryRight =
      new TalonFX(FLYWHEEL_SECONDARY_RIGHT.getDeviceNumber(), FLYWHEEL_SECONDARY_RIGHT.getCANBus());
  private final TalonFX secondaryLeft =
      new TalonFX(FLYWHEEL_SECONDARY_LEFT.getDeviceNumber(), FLYWHEEL_SECONDARY_LEFT.getCANBus());
  private final TalonFX feeder = new TalonFX(FLYWHEEL_FEED.getDeviceNumber());
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {
    FLYWHEEL_LEADER.getPowerPort(),
    FLYWHEEL_FOLLOWER.getPowerPort(),
    FLYWHEEL_SECONDARY_RIGHT.getPowerPort(),
    FLYWHEEL_SECONDARY_LEFT.getPowerPort(),
    FLYWHEEL_FEED.getPowerPort()
  };

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Current> secondaryRightCurrent = secondaryRight.getSupplyCurrent();
  private final StatusSignal<Current> secondaryLeftCurrent = secondaryLeft.getSupplyCurrent();
  private final StatusSignal<Current> feederCurrent = feeder.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration followerConfig;
  private final TalonFXConfiguration secondaryRightConfig;
  private final TalonFXConfiguration secondaryLeftConfig;
  private final TalonFXConfiguration feedConfig;

  public FlywheelIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kFlywheelIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kFlywheelOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kFlywheelOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kFlywheelOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kFlywheelClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kFlywheelClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kFlywheelClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);

    feedConfig = config.clone();
    if (kFlywheelFeedInverted)
      feedConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else feedConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    followerConfig = config.clone();
    if (kFlywheelFollowerInverted)
      followerConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else followerConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    secondaryRightConfig = config.clone();
    secondaryLeftConfig = config.clone();

    if (kFlywheelLeaderInverted)
      config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    // Apply the configurations to the flywheel motors
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(followerConfig);
    secondaryRight.getConfigurator().apply(secondaryRightConfig);
    secondaryLeft.getConfigurator().apply(secondaryLeftConfig);
    feeder.getConfigurator().apply(feedConfig);

    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
    secondaryRight.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
    secondaryLeft.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    leaderAppliedVolts.setUpdateFrequency(200.0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderCurrent,
        followerCurrent,
        secondaryRightCurrent,
        secondaryLeftCurrent,
        feederCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / kFlywheelGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / kFlywheelGearRatio;
    inputs.velocityRPM = leaderVelocity.getValueAsDouble() * 60 / kFlywheelGearRatio;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {
          leaderCurrent.getValueAsDouble(),
          followerCurrent.getValueAsDouble(),
          secondaryRightCurrent.getValueAsDouble(),
          secondaryLeftCurrent.getValueAsDouble(),
          feederCurrent.getValueAsDouble()
        };
    inputs.kP = config.Slot0.kP;
    inputs.kI = config.Slot0.kI;
    inputs.kD = config.Slot0.kD;
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setSpeed(double speed) {
    leader.set(speed);
  }

  @Override
  public void runFeed(double speed) {
    feeder.set(speed);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    feeder.stopMotor();
  }

  @Override
  public void stopFeed() {
    feeder.stopMotor();
  }

  /**
   * Set the PID portion of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   */
  @Override
  public void configurePID(double kP, double kI, double kD, int motorNum) {

    switch (motorNum) {
      case 1:
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        break;
      case 2:
        followerConfig.Slot0.kP = kP;
        followerConfig.Slot0.kI = kI;
        followerConfig.Slot0.kD = kD;
        break;
      case 3:
        secondaryRightConfig.Slot0.kP = kP;
        secondaryRightConfig.Slot0.kI = kI;
        secondaryRightConfig.Slot0.kD = kD;
        break;
      case 4:
        secondaryLeftConfig.Slot0.kP = kP;
        secondaryLeftConfig.Slot0.kI = kI;
        secondaryLeftConfig.Slot0.kD = kD;
        break;
    }
  }

  /**
   * Set the FeedForward portion of the Slot0 closed-loop configuration
   *
   * @param kS Static gain
   * @param kV Velocity gain
   */
  @Override
  public void configureFF(double kS, double kV) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }

  /**
   * Set the FeedForward portion of the Slot0 closed-loop configuration
   *
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  public void configureFF(double kS, double kV, double kA, int motorNum) {

    switch (motorNum) {
      case 1:
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        break;
      case 2:
        followerConfig.Slot0.kS = kS;
        followerConfig.Slot0.kV = kV;
        followerConfig.Slot0.kA = kA;
        break;
      case 3:
        secondaryRightConfig.Slot0.kS = kS;
        secondaryRightConfig.Slot0.kV = kV;
        secondaryRightConfig.Slot0.kA = kA;
        break;
      case 4:
        secondaryLeftConfig.Slot0.kS = kS;
        secondaryLeftConfig.Slot0.kV = kV;
        secondaryLeftConfig.Slot0.kA = kA;
        break;
    }
  }

  @Override
  public void configureAll() {
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(followerConfig, 0.25));
  }

  @Override
  public double getKP() {
    return config.Slot0.kP;
  }

  @Override
  public double getKI() {
    return config.Slot0.kI;
  }

  @Override
  public double getKD() {
    return config.Slot0.kD;
  }
}
