package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    TalonFX leftRollerMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_MOTOR_ID);
    TalonFX rightRollerMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_MOTOR_ID);
    TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration();

    SparkFlex deployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    PIDController deployPid = new PIDController(5.0, 0.0, 0.0);

    public Intake() {
        // Left roller
        leftRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Right roller (inverted)
        rightRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftRollerMotor.getConfigurator().apply(leftRollerConfig);
        rightRollerMotor.getConfigurator().apply(rightRollerConfig);

        // Deploy (Vortex, unchanged)
        SparkFlexConfig deployMotorConfig = new SparkFlexConfig();
        deployMotorConfig.smartCurrentLimit(40);
        deployMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        deployMotorConfig.inverted(true);
        deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployMotor.getEncoder().setPosition(0);

        initializePreferences();
    }

    public void setIntakeRollerMotorVoltage(double voltage) {
        leftRollerMotor.setVoltage(voltage);
        rightRollerMotor.setVoltage(voltage);
    }

    public void setDeployMotorVoltage(double voltage) {
        deployMotor.setVoltage(voltage);
    }

    public void resetIntakeEncoder() {
        deployMotor.getEncoder().setPosition(0);
    }

    /**
     * @param angle target angle in radians. 0 is hard stop, + is CCW
     */
    public void setDeployMotor(double angle) {
        double currentAngle = deployMotor.getEncoder().getPosition();
        deployMotor.setVoltage(deployPid.calculate(currentAngle, angle));
    }

    private void initializePreferences() {
        Preferences.initDouble("Intake/kP", 0.0);
        Preferences.initDouble("Deploy/TARGET_ANGLE", 0.0);
    }

    public void resetDeployMotor() {
        deployMotor.getEncoder().setPosition(0);
    }

    public void refreshPreferences() {
        deployPid.setP(Preferences.getDouble("Intake/kP", 0.0));
    }

    public void resetPID() {
        deployPid.reset();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/DEPLOY_MOTOR_POSITION", deployMotor.getEncoder().getPosition());
        // TalonFX velocity is rotations/sec; *60 gives RPM like the Spark encoder did
        Logger.recordOutput("Intake/LEFT_ROLLOR_MOTOR_VELOCITY", leftRollerMotor.getVelocity().getValueAsDouble() * 60.0);
        Logger.recordOutput("Intake/RIGHT_ROLLOR_MOTOR_VELOCITY", rightRollerMotor.getVelocity().getValueAsDouble() * 60.0);
    }
}