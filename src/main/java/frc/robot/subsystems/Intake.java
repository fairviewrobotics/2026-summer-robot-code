package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // "Left" meaning +y
    SparkFlex leftRollerMotor = new SparkFlex(IntakeConstants.INTAKE_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlex rightRollerMotor = new SparkFlex(IntakeConstants.INTAKE_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlex deployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    double voltage;
    private final SparkClosedLoopController deployController =
            deployMotor.getClosedLoopController();
    private final SparkFlexConfig deployConfig =
            new SparkFlexConfig();


    public Intake() {
        /*TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftRollerMotor.getConfigurator().apply(rollerMotorConfig);
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightRollerMotor.getConfigurator().apply(rollerMotorConfig);
        deployMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        deployMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        deployMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        deployMotorConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.INTAKE_DEPLOY_ABSOLUTE_ENCODER_ID;
        deployMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        deployMotorConfig.Feedback.RotorToSensorRatio = 1.0;
        deployMotorConfig.Slot0.kP = 0.0;
        deployMotorConfig.Slot0.kD = 0.0;*/
        //deployMotor.getConfigurator().apply(deployMotorConfig);
        this.voltage = Preferences.getDouble("Intake/voltage", 0.0);
    }

    public void setIntakeRollerMotorVoltage(double voltage) {
        leftRollerMotor.setVoltage(voltage);
        rightRollerMotor.setVoltage(voltage);
    }

    public void setIntakeRollerMotorVoltageWithPrefrances(){
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
        double rotations = Units.radiansToRotations(angle);
        deployController.setReference(
                rotations,
                SparkBase.ControlType.kPosition
        );

    }

    private void initializePreferences() {
        Preferences.initDouble("Intake/kP", 0.0);
        Preferences.initDouble("Intake/kD", 0.0);
        Preferences.initDouble("Deploy/TARGET_ANGLE", 0.0);
    }

    private void refreshPreferences() {
        TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
        deployMotorConfig.Slot0.kP = Preferences.getDouble("Intake/kP", 0.0);
        deployMotorConfig.Slot0.kD = Preferences.getDouble("Intake/kD", 0.0);
        deployMotor.getConfigurator().apply(deployMotorConfig);
    }

    public void periodic(){
        /*Logger.recordOutput("Intake/DEPLOY_MOTOR_POSITION", Units.rotationsToRadians(deployMotor.getPosition().getValueAsDouble()));
        Logger.recordOutput("Intake/LEFT_ROLLOR_MOTOR_POSITION", Units.rotationsToRadians(leftRollerMotor.getPosition().getValueAsDouble()));
        Logger.recordOutput("Intake/RIGHT_ROLLOR_MOTOR_POSITION", Units.rotationsToRadians(rightRollerMotor.getPosition().getValueAsDouble()));
        Logger.recordOutput("Intake/LEFT_ROLLOR_MOTOR_VELOCITY", leftRollerMotor.getVelocity().getValueAsDouble() * 16);
        Logger.recordOutput("Intake/RIGHT_ROLLOR_MOTOR_VELOCITY", rightRollerMotor.getVelocity().getValueAsDouble() * 16);*/
    }

}
