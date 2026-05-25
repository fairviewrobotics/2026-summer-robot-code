package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.SwerveModuleConfig;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final CANcoder turningEncoder;
  private final SwerveModuleConfig config;

  // --- Sim state handles (CTRE) ---
  private final TalonFXSimState driveSim;
  private final TalonFXSimState turnSim;
  private final CANcoderSimState encoderSim;

  // --- WPILib plant models ---
  // These model the physical inertia; TalonFXSimState feeds voltage in and reads velocity/position out.
  private final DCMotorSim drivePhysicsSim;
  private final DCMotorSim turnPhysicsSim;

  public SwerveModule(SwerveModuleConfig moduleConfig) {

    driveMotor    = new TalonFX(moduleConfig.driveMotorID());
    turningMotor  = new TalonFX(moduleConfig.turnMotorID());
    turningEncoder = new CANcoder(moduleConfig.encoderID());
    this.config   = moduleConfig;

    // Grab sim state handles from the hardware objects — these are the bridge
    // between the WPILib physics sim and CTRE's internal closed-loop sim.
    driveSim   = driveMotor.getSimState();
    turnSim    = turningMotor.getSimState();
    encoderSim = turningEncoder.getSimState();

    // DCMotorSim replaces both FlywheelSim and the old DCMotorSim.
    // It integrates position automatically, so we no longer need driveSimPositionRotations.
    drivePhysicsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.025,                              // drive wheel + gearbox moment of inertia (kg·m²)
            SwerveConstants.DRIVE_GEAR_RATIO
        ),
        DCMotor.getKrakenX60(1)
    );

    turnPhysicsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.001,                              // steer module moment of inertia (kg·m²)
            SwerveConstants.TURNING_GEAR_RATIO
        ),
        DCMotor.getKrakenX60(1)
    );

    // ---- Motor configuration (unchanged) ----
    TalonFXConfiguration driveConfig   = new TalonFXConfiguration();
    TalonFXConfiguration turningConfig = new TalonFXConfiguration();

    turningConfig.Slot0.kP = SwerveConstants.TURNING_P;
    turningConfig.Slot0.kD = SwerveConstants.TURNING_D;
    turningConfig.Feedback.FeedbackSensorSource    = FeedbackSensorSourceValue.FusedCANcoder;
    turningConfig.Feedback.FeedbackRemoteSensorID  = moduleConfig.encoderID();
    turningConfig.Feedback.SensorToMechanismRatio  = 1.0;
    turningConfig.Feedback.RotorToSensorRatio      = SwerveConstants.TURNING_GEAR_RATIO;
    turningConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turningConfig.MotorOutput.NeutralMode          = NeutralModeValue.Brake;
    turningConfig.MotorOutput.Inverted             = moduleConfig.turnInverted()
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    turningConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
    turningConfig.CurrentLimits.SupplyCurrentLimit       = 30.0;
    turningConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turningConfig.CurrentLimits.SupplyCurrentLowerLimit  = 50.0;
    turningConfig.CurrentLimits.SupplyCurrentLowerTime   = 1.0;
    turningConfig.CurrentLimits.StatorCurrentLimit       = 80.0;
    turningConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.Slot0.kP = SwerveConstants.DRIVE_P;
    driveConfig.Slot0.kD = SwerveConstants.DRIVE_D;
    driveConfig.Slot0.kV = SwerveConstants.DRIVE_KV;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.MotorOutput.Inverted    = moduleConfig.driveInverted()
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    driveConfig.CurrentLimits.SupplyCurrentLimit       = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit  = 80.0;
    driveConfig.CurrentLimits.SupplyCurrentLowerTime   = 2.0;
    driveConfig.CurrentLimits.StatorCurrentLimit       = 100.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    setAbsoluteEncoderOffset(moduleConfig.absoluteEncoderOffset());
    driveMotor.getConfigurator().apply(driveConfig);
    turningMotor.getConfigurator().apply(turningConfig);

    Preferences.initDouble(moduleConfig.name() + "/Offset", moduleConfig.absoluteEncoderOffset());
  }

  // ---- Public API (unchanged) ----

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getVelocity().getValueAsDouble()
            / SwerveConstants.DRIVE_GEAR_RATIO
            * SwerveConstants.WHEEL_CIRCUMFERENCE_METERS,
        Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble()
            / SwerveConstants.DRIVE_GEAR_RATIO
            * SwerveConstants.WHEEL_CIRCUMFERENCE_METERS,
        Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = Rotation2d.fromRotations(
        turningEncoder.getAbsolutePosition().getValueAsDouble());

    desiredState.optimize(encoderRotation);
    desiredState.cosineScale(encoderRotation);

    driveMotor.setControl(new VelocityVoltage(
        desiredState.speedMetersPerSecond
            / SwerveConstants.WHEEL_CIRCUMFERENCE_METERS
            * SwerveConstants.DRIVE_GEAR_RATIO));
    turningMotor.setControl(new PositionVoltage(desiredState.angle.getRotations()));
  }

  public void setAbsoluteEncoderOffset(double offset) {
    CANcoderConfiguration turningEncoderConfig = new CANcoderConfiguration();
    turningEncoderConfig.MagnetSensor.MagnetOffset = Units.degreesToRotations(offset);
    turningEncoder.getConfigurator().apply(turningEncoderConfig);
  }

  public double getClosedLoopOutput()   { return driveMotor.getClosedLoopOutput().getValueAsDouble(); }
  public double getDriveMotorCurrent()  { return driveMotor.getSupplyCurrent().getValueAsDouble(); }

  public void latchPosition() {
    turningMotor.setControl(new PositionVoltage(
        turningEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  public void refreshPreferences() {
    double offset = Preferences.getDouble(config.name() + "/Offset", config.absoluteEncoderOffset());
    setAbsoluteEncoderOffset(offset);

    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.Slot0.kP = Preferences.getDouble("Swerve/TurningP", SwerveConstants.TURNING_P);
    turningConfig.Slot0.kD = Preferences.getDouble("Swerve/TurningD", SwerveConstants.TURNING_D);
    turningMotor.getConfigurator().apply(turningConfig);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = Preferences.getDouble("Swerve/DriveP",  SwerveConstants.DRIVE_P);
    driveConfig.Slot0.kD = Preferences.getDouble("Swerve/DriveD",  SwerveConstants.DRIVE_D);
    driveConfig.Slot0.kV = Preferences.getDouble("Swerve/DriveKV", SwerveConstants.DRIVE_KV);
    driveMotor.getConfigurator().apply(driveConfig);
  }


  @Override
  public void simulationPeriodic() {

    driveSim.setSupplyVoltage(12.0);
    turnSim.setSupplyVoltage(12.0);

    drivePhysicsSim.setInputVoltage(driveSim.getMotorVoltage());
    drivePhysicsSim.update(0.02);

    driveSim.setRawRotorPosition(
        Units.radiansToRotations(drivePhysicsSim.getAngularPositionRad())
            * SwerveConstants.DRIVE_GEAR_RATIO);
    driveSim.setRotorVelocity(
        Units.radiansToRotations(drivePhysicsSim.getAngularVelocityRadPerSec())
            * SwerveConstants.DRIVE_GEAR_RATIO);

    turnPhysicsSim.setInputVoltage(turnSim.getMotorVoltage());
    turnPhysicsSim.update(0.02);

    turnSim.setRawRotorPosition(
        Units.radiansToRotations(turnPhysicsSim.getAngularPositionRad())
            * SwerveConstants.TURNING_GEAR_RATIO);
    turnSim.setRotorVelocity(
        Units.radiansToRotations(turnPhysicsSim.getAngularVelocityRadPerSec())
            * SwerveConstants.TURNING_GEAR_RATIO);

    encoderSim.setRawPosition(
        Units.radiansToRotations(turnPhysicsSim.getAngularPositionRad()));
    encoderSim.setVelocity(
        Units.radiansToRotations(turnPhysicsSim.getAngularVelocityRadPerSec()));
  }
}