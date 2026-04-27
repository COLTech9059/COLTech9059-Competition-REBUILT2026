package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.extendedPositionDeadbandInRadians;
import static frc.robot.Constants.IntakeConstants.extendedPositionInRadians;
import static frc.robot.Constants.IntakeConstants.ffIntakeRollers;
import static frc.robot.Constants.IntakeConstants.ffPosition;
import static frc.robot.Constants.IntakeConstants.ffPositionFollower;
import static frc.robot.Constants.IntakeConstants.pidIntakeRollers;
import static frc.robot.Constants.IntakeConstants.pidPosition;
import static frc.robot.Constants.IntakeConstants.pidPositionFollower;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem, driven by several motors; Kraken, NEO, and hybrid compatibility
 *
 * @author DevAspen
 */
public class Intake extends RBSISubsystem {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private final SysIdRoutine sysIdRollers;

  private double velocitySetpointRPM = 0;

  public Intake(IntakeIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
        io.setPID(pidPosition.kP, pidPosition.kI, pidPosition.kD, 1);
        io.setFF(ffPosition[0], ffPosition[1], ffPosition[2], 1);
        io.setPID(pidPositionFollower.kP, pidPositionFollower.kI, pidPositionFollower.kD, 2);
        io.setFF(ffPositionFollower[0], ffPositionFollower[1], ffPositionFollower[2], 2);
        io.setPID(pidIntakeRollers.kP, pidIntakeRollers.kD, pidIntakeRollers.kD, 3);
        io.setFF(ffIntakeRollers[0], ffIntakeRollers[1], ffIntakeRollers[2], 3);
        io.configureAll();
        break;
      case SIM:
      case REPLAY:
        io.setPID(pidPosition.kP, pidPosition.kI, pidPosition.kD, 1);
        io.setFF(ffPosition[0], ffPosition[1], ffPosition[2], 1);
        io.setPID(pidPositionFollower.kP, pidPositionFollower.kI, pidPositionFollower.kD, 2);
        io.setFF(ffPositionFollower[0], ffPositionFollower[1], ffPositionFollower[2], 2);
        io.setPID(pidIntakeRollers.kP, pidIntakeRollers.kD, pidIntakeRollers.kD, 3);
        io.setFF(ffIntakeRollers[0], ffIntakeRollers[1], ffIntakeRollers[2], 3);
        io.configureAll();
        break;
    }

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(1.0),
                Time.ofBaseUnits(5.0, edu.wpi.first.units.Units.Seconds),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runPositionVolts(voltage.in(Volts)), null, this));

    sysIdRollers =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Volts.of(1).per(Second),
                null, // Volts.of(8.0),
                Time.ofBaseUnits(7.0, edu.wpi.first.units.Units.Seconds),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Total Current", getTotalCurrent());

    // Log velocity setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocitySetpointRPM);
  }

  /** Returns the total current draw of the subsystem */
  public double getTotalCurrent() {
    double currentSum = 0;
    for (int i = 0; i < inputs.currentAmps.length; i++) {
      currentSum += Math.abs(inputs.currentAmps[i]);
    }
    return currentSum;
  }

  /** Run the positioning motors at the given voltage */
  public void runPositionVolts(double volts) {
    io.setPositionVoltage(volts);
  }

  /** Run the intake rollers at the specified voltage */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run the intake rollers at the specified speed */
  public void runSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Run the positioning motors at the given speed */
  public void runPositionSpeed(double positionSpeed) {
    io.setPositionSpeed(positionSpeed);
  }

  /** Run the feed motor at the given speed */
  public void runFeed(double speed) {
    io.runFeed(speed);
  }

  /** Set the position of the intake, either extended or retracted */
  public void setPosition(double baseSpeed, boolean out) {
    io.setPosition(baseSpeed, out);

    Logger.recordOutput("Intake/setpointPosition", out);
  }

  public void setIntakeTorque(double torqueSetpoint) {
    io.setIntakeTorque(torqueSetpoint);
  }

  /**
   * Set the velocity of the intake & feed system
   *
   * @param velocityRPM The velocity setpoint of the intake rollers in RPM
   * @param feedSpeed The speed to set the feed system to
   */
  public void setIntakeVelocity(double velocityRPM, double feedSpeed) {
    io.setIntakeVelocity(velocityRPM, feedSpeed);

    velocitySetpointRPM = velocityRPM;
  }

  /** Set the position of the positioning system in degrees; closed loop */
  public void setPosition(double positionDegrees) {
    io.setPosition(positionDegrees);
  }

  /** Jostle/oscillate the intake with the given base speed */
  public void jostleIntake(double baseSpeed) {
    double intakePosition = Units.degreesToRadians(inputs.positionDegrees);
    boolean yesReverse =
        (intakePosition >= (extendedPositionInRadians - (2 * extendedPositionDeadbandInRadians)));
    io.setPosition(baseSpeed, yesReverse);
  }

  /** Stop the intake rollers */
  public void stopIntake() {
    io.stopIntake();
  }

  /** Stop the intake position motor */
  public void stopPosition() {
    io.stopPosition();
  }

  /** Get the intake position in degrees */
  public double getIntakePos() {
    return inputs.positionDegrees;
  }

  /** Returns true if the intake is in the "out" position */
  public boolean isIntakeOut() {
    double currentAngle = Units.degreesToRadians(inputs.positionDegrees);
    return ((currentAngle >= (extendedPositionInRadians - extendedPositionDeadbandInRadians)));
  }

  /** Returns true if the intake is in the "in" position */
  public boolean isIntakeIn() {
    return (isIntakeInLeft() || isIntakeInRight());
  }

  /** Return true if the left intake sensor is active. */
  public boolean isIntakeInLeft() {
    return inputs.isIntakeInLeft;
  }

  /** Return true if the intake is 'in' */
  public boolean isIntakeInRight() {
    return inputs.isIntakeInRight;
  }

  /** Sets the speed of the intake and feed system to the given speeds */
  public void setSpeed(double intakeSpeed, double feedSpeed) {
    io.setSpeed(intakeSpeed, feedSpeed);
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns a command to run a quasistatic intake roller test in the specified direction */
  public Command sysIdQuasistaticIntake(SysIdRoutine.Direction direction) {
    return sysIdRollers.quasistatic(direction);
  }

  /** Returns a command to run a dynamic intake roller test in the specified direction */
  public Command sysIdDynamicIntake(SysIdRoutine.Direction direction) {
    return sysIdRollers.dynamic(direction);
  }
}
