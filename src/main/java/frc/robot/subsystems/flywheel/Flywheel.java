// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FlywheelConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel subsystem, driven by several motors; Kraken, NEO, and hybrid compatibility; open &
 * closed-loop control
 */
public class Flywheel extends RBSISubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private double variableSpeed = minFlywheelSpeed;

  private Timer spinUpTimer = new Timer();

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        io.configurePID(pid1Real.kP, pid1Real.kI, pid1Real.kD, 1);
        io.configureFF(ff1Real[0], ff1Real[1], ff1Real[2], 1);
        io.configurePID(pid2Real.kP, pid2Real.kI, pid2Real.kD, 2);
        io.configureFF(ff2Real[0], ff2Real[1], ff2Real[2], 2);
        io.configurePID(pid3Real.kP, pid3Real.kI, pid3Real.kD, 3);
        io.configureFF(ff3Real[0], ff3Real[1], ff3Real[2], 3);
        io.configurePID(pid4Real.kP, pid4Real.kI, pid4Real.kD, 4);
        io.configureFF(ff4Real[0], ff4Real[1], ff4Real[2], 4);
        io.configureAll();
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(ff1Real[0], ff1Real[1], ff1Real[2]);
        io.configurePID(pid1Real.kP, pid1Real.kI, pid1Real.kD, 1);
        io.configureFF(ff1Real[0], ff1Real[1], ff1Real[2], 1);
        io.configurePID(pid2Real.kP, pid2Real.kI, pid2Real.kD, 2);
        io.configureFF(ff2Real[0], ff2Real[1], ff2Real[2], 2);
        io.configurePID(pid3Real.kP, pid3Real.kI, pid3Real.kD, 3);
        io.configureFF(ff3Real[0], ff3Real[1], ff3Real[2], 3);
        io.configurePID(pid4Real.kP, pid4Real.kI, pid4Real.kD, 4);
        io.configureFF(ff4Real[0], ff4Real[1], ff4Real[2], 4);
        io.configureAll();
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(kStaticGainSim, kVelocityGainSim);
        io.configurePID(pidSim.kP, pidSim.kI, pidSim.kD, 1);
        io.configureFF(kStaticGainSim, kVelocityGainSim, kAccelerationGainSim, 1);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(8),
                Time.ofBaseUnits(8.0, edu.wpi.first.units.Units.Seconds),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    SmartDashboard.putNumber("Flywheel Variable Speed", variableSpeed);
    SmartDashboard.putBoolean("Flywheel Spun Up?", isTimerPastValue(flywheelSpinUpTime));

    Logger.recordOutput("Flywheel/Total Current", getTotalCurrent());
  }

  public double getTotalCurrent() {
    double currentSum = 0;
    for (int i = 0; i < inputs.currentAmps.length; i++) {
      currentSum += inputs.currentAmps[i];
    }
    return currentSum;
  }

  public void startTimer() {
    spinUpTimer.reset();
    spinUpTimer.start();
  }

  public double getTimer() {
    return spinUpTimer.get();
  }

  public boolean isTimerPastValue(double time) {
    return spinUpTimer.get() >= time;
  }

  public void stopTimer() {
    spinUpTimer.stop();
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Run flywheel using a given distance. Used for shooting at the Hub. */
  public void runVelocityBasedOnDistance(double inputDistance) {}

  /** Run the feed system at the specified speed */
  public void runFeed(double speed) {
    io.runFeed(speed);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setSpeed() {
    io.setSpeed(variableSpeed);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Stop the feed system */
  public void stopFeed() {
    io.stopFeed();
  }

  public void incrementSpeed(boolean increase) {
    if (increase)
      variableSpeed = Math.min(maxFlywheelSpeed, variableSpeed + flywheelSpeedIncrement);
    else variableSpeed = Math.max(minFlywheelSpeed, variableSpeed - flywheelSpeedIncrement);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Flywheel")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean isFlywheelUpToSpeed(double setpointRPM, double tolerance) {
    double velocityRotationsPerSec = Units.radiansToRotations(inputs.velocityRadPerSec);
    return velocityRotationsPerSec <= setpointRPM * (1 + tolerance)
        && velocityRotationsPerSec >= (1 - tolerance);
  }

  public double getFlywheelRPMFromDistance(double distanceFromTarget) {
    double targetVelocity =
        velocityMultiplier
            * (1.10926 * distanceFromTarget
                + 14.23834); // Calculate required launch velocity in feet/second
    double targetRPM =
        Units.radiansPerSecondToRotationsPerMinute(
            targetVelocity / 4.0); // Calculate target velocity in Radians/sec, then convert to RPM
    return targetRPM;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }

  public void configurePID(double kP, double kI, double kD, int motorNum) {
    io.configurePID(kP, kI, kD, motorNum);
  }

  public void configureAll() {
    io.configureAll();
  }

  public double getKP() {
    return io.getKP();
  }

  public double getKI() {
    return io.getKI();
  }

  public double getKD() {
    return io.getKD();
  }
}
