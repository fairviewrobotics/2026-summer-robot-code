package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

    public SparkFlex hoodMotor = new SparkFlex(ShootingConstants.HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public SparkFlexConfig hoodMotorConfig = new SparkFlexConfig();

    private final InterpolatingDoubleTreeMap DistanceToAngle =
            new InterpolatingDoubleTreeMap();

    private PIDController hoodPID = new PIDController(0.0, 0.0, 0.0);

    public Hood() {
        initializePreferences();
        resetHoodPosition();

        hoodMotorConfig.smartCurrentLimit(20);

        hoodMotorConfig.encoder.positionConversionFactor(ShootingConstants.HOOD_CONVERSION_FACTOR);

        hoodMotorConfig.inverted(false);
        hoodMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        createDistanceToAngleMap();
        hoodPID.setTolerance(0.01);
    }

    private void initializePreferences() {
        Preferences.initDouble("Hood/kP", 0.0);
        Preferences.initDouble("Hood/kD", 0.0);
        Preferences.initDouble("Hood/TARGET_ANGLE", 0.0);
    }

    public void updatePreferences() {
        hoodPID.setP(Preferences.getDouble("Hood/kP", 0.0));
        hoodPID.setD(Preferences.getDouble("Hood/kD", 0.0));
    }


    /**
     *
     * @param position in radians
     */

    public void setHoodPosition(double position) {
        hoodMotor.setVoltage(hoodPID.calculate(hoodMotor.getEncoder().getPosition(), position));
    }

    public void setHoodVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    public void periodic() {
        Logger.recordOutput("Hood/HOOD_POSITION", hoodMotor.getEncoder().getPosition());
    }

    public void stopHood() {
        hoodMotor.stopMotor();
    }

    public void setVoltage(double voltage) {hoodMotor.setVoltage(voltage);}

    public void resetHoodPosition() {
        hoodMotor.getEncoder().setPosition(0);
    }

    private void createDistanceToAngleMap() {
        DistanceToAngle.put(0.0, Math.toRadians(10));
        DistanceToAngle.put(3.0796, Math.toRadians(20));
        DistanceToAngle.put(4.1596, Math.toRadians(40));
        DistanceToAngle.put(5.1396, Math.toRadians(60));
        DistanceToAngle.put(6.0, Math.toRadians(80));
    }

    public double getDistanceToAngle(double distance){
        return DistanceToAngle.get(distance);
    }

    public void setVoltage(Double voltage) {
        hoodMotor.setVoltage(voltage);
    }



}
