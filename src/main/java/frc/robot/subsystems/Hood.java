package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

    public TalonFX hoodMotor = new TalonFX(ShootingConstants.HOOD_MOTOR_ID);
    public TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();

    private final InterpolatingDoubleTreeMap DistanceToAngle =
            new InterpolatingDoubleTreeMap();

    public Hood() {
        initializePreferences();
        resetHoodPosition();

        hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        hoodMotorConfig.Feedback.SensorToMechanismRatio = ShootingConstants.HOOD_CONVERSION_FACTOR;

        hoodMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hoodMotorConfig.Slot0.kP = Preferences.getDouble("Hood/kP", 0.0);
        hoodMotorConfig.Slot0.kD = Preferences.getDouble("Hood/kD", 0.0);
        createDistanceToAngleMap();
    }

    private void initializePreferences() {
        Preferences.initDouble("Hood/kP", 0.0);
        Preferences.initDouble("Hood/kD", 0.0);
        Preferences.initDouble("Hood/TARGET_ANGLE", 0.0);
    }

    private void updatePreferences() {
        Slot0Configs hoodMotorConfig = new Slot0Configs();
        hoodMotorConfig.kP = Preferences.getDouble("Hood/kP", 0.0);
        hoodMotorConfig.kD = Preferences.getDouble("Hood/kD", 0.0);
        hoodMotor.getConfigurator().apply(hoodMotorConfig);
    }


    /**
     *
     * @param position in degrees
     */

    public void setHoodPosition(double position) {
        double positionRadians = Units.degreesToRadians(position);

        hoodMotor.setControl(new PositionVoltage(positionRadians));
    }

    public void periodic() {
        Logger.recordOutput("Hood/HOOD_POSITION", hoodMotor.getPosition().getValueAsDouble());
    }

    public void stopHood() {
        hoodMotor.stopMotor();
    }

    public void resetHoodPosition() {
        hoodMotor.setPosition(0);
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



}
