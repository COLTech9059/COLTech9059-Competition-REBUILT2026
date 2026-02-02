package frc.robot.subsystems.climber;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ClimberConstants.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climbMotor = new TalonFX(CLIMBER_MOTOR.getDeviceNumber());
  private DigitalInput upLimit = new DigitalInput(CLIMBER_UP_LIMIT);
  private DigitalInput downLimit = new DigitalInput(CLIMBER_DOWN_LIMIT);
  public final int[] powerPorts = {
    CLIMBER_MOTOR.getPowerPort()
  };

  private final StatusSignal<Angle> position = climbMotor.getPosition();
  private final StatusSignal<Current> climberCurrent = climbMotor.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ClimberIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kClimberIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kClimberOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kClimberOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kClimberOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kClimberClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kClimberClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kClimberClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);
    
    if (kClimberInverted) config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    else config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    climbMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, climberCurrent);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, climberCurrent);

    inputs.climberSpeed = climbMotor.get() * 100.0;
    inputs.positionInches = position.getValueAsDouble() / kClimberRotationsToInchesRatio;
    inputs.isClimberUp = upLimit.get();
    inputs.isClimberDown = downLimit.get();
    inputs.currentAmps = new double[] {climberCurrent.getValueAsDouble()};

    // AdvantageKit logging
    // Logger.recordOutput("Climber/ClimberSpeed", inputs.climberSpeed);
    // Logger.recordOutput("Climber/PositionInches", inputs.positionInches);
    // Logger.recordOutput("Climber/IsClimberUp", inputs.isClimberUp);
    // Logger.recordOutput("Climber/IsClimberDown", inputs.isClimberDown);
    // Logger.recordOutput("Climber/ClimberCurrent", inputs.currentAmps);
  }

  @Override
  public void setSpeed(double speed) {
    climbMotor.set(speed);
  }

  @Override
  public void setPosition(double speed, boolean up) {
    speed = Math.abs(speed);
    
    if (up) {
      if (upLimit.get()) stop();
      else setSpeed(speed);
    }
    else {
      if (downLimit.get()) stop();
      else setSpeed(-speed);
    }
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
  }
}
