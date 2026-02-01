package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalon implements IntakeIO {

  private TalonFX positionMotor = new TalonFX(INTAKE_POSITION_MOTOR.getDeviceNumber());
  private TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR.getDeviceNumber());
  private DigitalInput outLimit = new DigitalInput(INTAKE_OUT_LIMIT);
  private DigitalInput inLimit = new DigitalInput(INTAKE_IN_LIMIT);
  public final int[] powerPorts = {
    INTAKE_POSITION_MOTOR.getPowerPort(), INTAKE_MOTOR.getPowerPort()
  };

  private final StatusSignal<Angle> position = positionMotor.getPosition();
  private final StatusSignal<Current> positionCurrent = positionMotor.getSupplyCurrent();
  private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();

  private final TalonFXConfiguration positionConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration intakeConfig;

  public IntakeIOTalon() {
    positionConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    positionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    positionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

    positionMotor.getConfigurator().apply(positionConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, positionCurrent, intakeCurrent);
    positionMotor.optimizeBusUtilization();
    intakeMotor.optimizeBusUtilization();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, positionCurrent, intakeCurrent);
    inputs.intakeSpeed = intakeMotor.get();
    inputs.positionDegrees =
        Units.rotationsToDegrees(position.getValueAsDouble()) / kIntakePositionGearRatio;
    inputs.isIntakeOut = isIntakeOut();
    inputs.currentAmps =
        new double[] {positionCurrent.getValueAsDouble(), intakeCurrent.getValueAsDouble()};
  }

  public void setVoltage(double volts) {
    intakeMotor.setControl(new VoltageOut(volts));
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

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

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void stopPosition() {
    positionMotor.stopMotor();
  }

  public double getIntakePos() {
    return Units.rotationsToDegrees(positionMotor.getPosition().getValueAsDouble())
        / kIntakePositionGearRatio;
  }

  public boolean isIntakeOut() {
    return outLimit.get();
  }
}
