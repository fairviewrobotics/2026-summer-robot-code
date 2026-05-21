package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.SwerveModuleConfig;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final CANcoder turningEncoder;
  private final SwerveModuleConfig config;

  public SwerveModule(SwerveModuleConfig moduleConfig) {

    driveMotor = new TalonFX(moduleConfig.driveMotorID());
    turningMotor = new TalonFX(moduleConfig.turnMotorID());
    turningEncoder = new CANcoder(moduleConfig.encoderID());
    this.config = moduleConfig;

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    TalonFXConfiguration turningConfig = new TalonFXConfiguration();

    turningConfig.Slot0.kP = 80.0;
    turningConfig.Slot0.kD = 0.5;
    turningConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turningConfig.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderID();
    turningConfig.Feedback.SensorToMechanismRatio = 1.0;
    turningConfig.Feedback.RotorToSensorRatio = SwerveConstants.TURNING_GEAR_RATIO;
    turningConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turningConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (moduleConfig.turnInverted()) {
      turningConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      turningConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    // OP Robotics Values:
    turningConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
    turningConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    turningConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turningConfig.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
    turningConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    turningConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    turningConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.Slot0.kP = 0.11;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.Slot0.kV = 0.109;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    if (moduleConfig.driveInverted()) {
      driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
    driveConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;
    driveConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    setAbsoluteEncoderOffset(moduleConfig.absoluteEncoderOffset());
    driveMotor.getConfigurator().apply(driveConfig);
    turningMotor.getConfigurator().apply(turningConfig);

    Preferences.initDouble(moduleConfig.name() + "/Offset", moduleConfig.absoluteEncoderOffset());
  }

    /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(turningMotor.getPosition().getValueAsDouble()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() / SwerveConstants.DRIVE_GEAR_RATIO * SwerveConstants.WHEEL_CIRCUMFERENCE_METERS,
            Rotation2d.fromRotations(turningMotor.getPosition().getValueAsDouble()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = Rotation2d.fromRotations(turningMotor.getPosition().getValueAsDouble());

    desiredState.optimize(encoderRotation);

    desiredState.cosineScale(encoderRotation);

    driveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond / SwerveConstants.WHEEL_CIRCUMFERENCE_METERS * SwerveConstants.DRIVE_GEAR_RATIO));
    turningMotor.setControl(new PositionDutyCycle(desiredState.angle.getRotations()));

  }

  public void setAbsoluteEncoderOffset(double offset) {
    CANcoderConfiguration turningEncoderConfig = new CANcoderConfiguration();
    turningEncoderConfig.MagnetSensor.MagnetOffset = Units.degreesToRotations(offset);
    turningEncoder.getConfigurator().apply(turningEncoderConfig);
  }

  public double getClosedLoopOutput() {
    return driveMotor.getClosedLoopOutput().getValueAsDouble();
  }

  public double getDriveMotorCurrent() {
      return driveMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void refreshPreferences() {
    double offset = Preferences.getDouble(config.name() + "/Offset", config.absoluteEncoderOffset());
    setAbsoluteEncoderOffset(offset);

    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.Slot0.kP = Preferences.getDouble("Swerve/TurningP", SwerveConstants.TURNING_P);
    turningConfig.Slot0.kD = Preferences.getDouble("Swerve/TurningD", SwerveConstants.TURNING_D);
    turningMotor.getConfigurator().apply(turningConfig);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = Preferences.getDouble("Swerve/DriveP", SwerveConstants.DRIVE_P);
    driveConfig.Slot0.kD = Preferences.getDouble("Swerve/DriveD", SwerveConstants.DRIVE_D);
    driveConfig.Slot0.kV = Preferences.getDouble("Swerve/DriveKV", SwerveConstants.DRIVE_KV);
    driveMotor.getConfigurator().apply(driveConfig);
  }

}
