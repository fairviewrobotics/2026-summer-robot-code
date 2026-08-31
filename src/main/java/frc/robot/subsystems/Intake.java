package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    SparkFlex leftRollerMotor = new SparkFlex(IntakeConstants.INTAKE_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlex rightRollerMotor = new SparkFlex(IntakeConstants.INTAKE_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlex deployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    PIDController deployPid = new PIDController(0.0, 0.0, 0.0);

    public Intake() {
        SparkFlexConfig rollerMotorConfig = new SparkFlexConfig();
        SparkFlexConfig deployMotorConfig = new SparkFlexConfig();
        rollerMotorConfig.smartCurrentLimit(40);
        rollerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rollerMotorConfig.inverted(false);
        leftRollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotorConfig.inverted(true);
        rightRollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployMotorConfig.smartCurrentLimit(40);
        deployMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        deployMotorConfig.inverted(false);
        deployMotorConfig.encoder.positionConversionFactor(2 * Math.PI);

        deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployMotor.getEncoder().setPosition(0);
    }

    public void setIntakeRollerMotorVoltage(double voltage) {
        leftRollerMotor.setVoltage(voltage);
        rightRollerMotor.setVoltage(voltage);
    }

    public void setDeployMotorVoltage(double voltage) {
        deployMotor.setVoltage(voltage);
    }

    /**
     *
     * @param angle target angle in radians. 0 is hard stop, + is CCW
     */

    public void setDeployMotor(double angle) {
        deployMotor.setVoltage(deployPid.calculate(angle));
    }

    private void initializePreferences() {
        Preferences.initDouble("Intake/kP", 0.0);
        Preferences.initDouble("Deploy/TARGET_ANGLE", 0.0);
    }

    public void refreshPreferences() {
        deployPid.setP(Preferences.getDouble("Intake/kP", 0.0));
    }

    public void periodic(){
        Logger.recordOutput("Intake/DEPLOY_MOTOR_POSITION", deployMotor.getEncoder().getPosition());
        Logger.recordOutput("Intake/LEFT_ROLLOR_MOTOR_VELOCITY", leftRollerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Intake/RIGHT_ROLLOR_MOTOR_VELOCITY", rightRollerMotor.getEncoder().getVelocity());
    }

}
