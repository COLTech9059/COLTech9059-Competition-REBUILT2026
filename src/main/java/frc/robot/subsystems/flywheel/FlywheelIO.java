// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO extends RBSIIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  public default void setSpeed(double speed) {}

  public default void runFeed(double speed) {}

  public default void stopFeed() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD, int motorNum) {}

  /** Set velocity FeedForward constants. */
  public default void configureFF(double kS, double kV) {}

  /** Set velocity FeedForward constants. */
  public default void configureFF(double kS, double kV, double kA, int motorNum) {}

  public default void configureAll() {}

  public default double getKP() {
    return 0;
  }

  public default double getKI() {
    return 0;
  }

  public default double getKD() {
    return 0;
  }
}
