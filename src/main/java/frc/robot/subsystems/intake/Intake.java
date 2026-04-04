package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
                (state) -> Logger.recordOutput("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runPositionVolts(voltage.in(Volts)),
                (sysIdRoutineLog) -> {
                  sysIdRoutineLog
                      .motor("Intake Leader")
                      .angularPosition(Angle.ofBaseUnits(inputs.positionLeader, Degrees))
                      .voltage(inputs.voltageLeader)
                      .angularVelocity(inputs.velocityLeader);
                  sysIdRoutineLog
                      .motor("Follower Leader")
                      .angularPosition(Angle.ofBaseUnits(inputs.positionFollower, Degrees))
                      .voltage(inputs.voltageFollower)
                      .angularVelocity(inputs.velocityFollower);
                },
                this));

    sysIdRollers =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(8.0),
                Time.ofBaseUnits(8.0, edu.wpi.first.units.Units.Seconds),
                (state) -> Logger.recordOutput("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVolts(voltage.in(Volts)),
                (sysIdRoutineLog) -> {
                  sysIdRoutineLog
                      .motor("Intake Rollers")
                      .angularPosition(Angle.ofBaseUnits(inputs.positionLeader, Degrees))
                      .voltage(inputs.voltageLeader)
                      .angularVelocity(inputs.velocityLeader);
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Total Current", getTotalCurrent());
  }

  public double getTotalCurrent() {
    double currentSum = 0;
    for (int i = 0; i < inputs.currentAmps.length; i++) {
      currentSum += inputs.currentAmps[i];
    }
    return currentSum;
  }

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

  public void runFeed(double speed) {
    io.runFeed(speed);
  }

  /** Set the position of the intake, either extended or retracted */
  public void setPosition(double baseSpeed, boolean out) {
    io.setPosition(baseSpeed, out);

    Logger.recordOutput("Intake/setpointPosition", out);
  }

  public void setIntakeVelocity(double velocityRPM, double feedSpeed) {
    io.setIntakeVelocity(velocityRPM, feedSpeed);
  }

  public void setPosition(double positionDegrees) {
    io.setPosition(positionDegrees);
  }

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

  public boolean isIntakeOut() {
    double currentAngle = Units.degreesToRadians(inputs.positionDegrees);
    return ((currentAngle >= (extendedPositionInRadians - extendedPositionDeadbandInRadians)));
  }

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

  public Command sysIdQuasistaticIntake(SysIdRoutine.Direction direction) {
    return sysIdRollers.quasistatic(direction);
  }

  public Command sysIdDynamicIntake(SysIdRoutine.Direction direction) {
    return sysIdRollers.quasistatic(direction);
  }
}
