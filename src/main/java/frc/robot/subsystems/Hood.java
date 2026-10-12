package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

    public final TalonFX hoodMotor = new TalonFX(ShootingConstants.HOOD_MOTOR_ID);

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final InterpolatingDoubleTreeMap DistanceToAngle = new InterpolatingDoubleTreeMap();

    // HOOD_CONVERSION_FACTOR = hood units (rad) per motor rotation, same as the old Spark config
    private static final double UNITS_PER_ROTATION = ShootingConstants.HOOD_CONVERSION_FACTOR;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public Hood() {
        initializePreferences();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // old: inverted(false)

        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Slot0 gains are volts per ROTOR ROTATION of error, so convert from per-radian gains
        config.Slot0.kP = Preferences.getDouble("Hood/kP", 0.0) * UNITS_PER_ROTATION;
        config.Slot0.kD = Preferences.getDouble("Hood/kD", 0.0) * UNITS_PER_ROTATION;

        hoodMotor.getConfigurator().apply(config);

        resetHoodPosition();
        createDistanceToAngleMap();
    }

    private void initializePreferences() {
        Preferences.initDouble("Hood/kP", 0.0);
        Preferences.initDouble("Hood/kD", 0.0);
        Preferences.initDouble("Hood/TARGET_ANGLE", 0.0);

        Preferences.initDouble("Hood_Map/DIST_MAP_ONE", 0.0);
        Preferences.initDouble("Hood_Map/DIST_MAP_TWO", 3.0796);
        Preferences.initDouble("Hood_Map/DIST_MAP_THREE", 4.1596);
        Preferences.initDouble("Hood_Map/DIST_MAP_FOUR", 5.1396);
        Preferences.initDouble("Hood_Map/DIST_MAP_FIVE", 6.0);

        Preferences.initDouble("Hood_Map/ANGLE_MAP_ONE", 10.0);
        Preferences.initDouble("Hood_Map/ANGLE_MAP_TWO", 20.0);
        Preferences.initDouble("Hood_Map/ANGLE_MAP_THREE", 0.0);
        Preferences.initDouble("Hood_Map/ANGLE_MAP_FOUR", 0.0);
        Preferences.initDouble("Hood_Map/ANGLE_MAP_FIVE", 80.0);
    }

    public void updatePreferences() {
        config.Slot0.kP = Preferences.getDouble("Hood/kP", 0.0) * UNITS_PER_ROTATION;
        config.Slot0.kD = Preferences.getDouble("Hood/kD", 0.0) * UNITS_PER_ROTATION;
        hoodMotor.getConfigurator().apply(config.Slot0);
        createDistanceToAngleMap();
    }

    /** @param position target in hood units (rad) */
    public void setHoodPosition(double position) {
        hoodMotor.setControl(positionRequest.withPosition(position / UNITS_PER_ROTATION));
    }

    public double getHoodPosition() {
        return hoodMotor.getPosition().getValueAsDouble() * UNITS_PER_ROTATION;
    }

    public boolean isAtPosition(double targetAngle, double toleranceRad) {
        return Math.abs(getHoodPosition() - targetAngle) <= toleranceRad;
    }

    public void setHoodVoltage(double voltage) {
        hoodMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public void stopHood() {
        hoodMotor.setControl(neutralRequest);
    }

    public void resetHoodPosition() {
        hoodMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        if (getHoodPosition() < 0) {
            resetHoodPosition();
        }
        Logger.recordOutput("Hood/HOOD_POSITION", getHoodPosition());
    }

    // Distance (m), Hood Angle (rad)
    private void createDistanceToAngleMap() {
        DistanceToAngle.clear();
        DistanceToAngle.put(Preferences.getDouble("Hood_Map/DIST_MAP_ONE", 0.0),
                Preferences.getDouble("Hood_Map/ANGLE_MAP_ONE", 10.0));
        DistanceToAngle.put(Preferences.getDouble("Hood_Map/DIST_MAP_TWO", 3.0796),
                Preferences.getDouble("Hood_Map/ANGLE_MAP_TWO", 20.0));
        DistanceToAngle.put(Preferences.getDouble("Hood_Map/DIST_MAP_THREE", 4.1596),
                Preferences.getDouble("Hood_Map/ANGLE_MAP_THREE", 0.0));
        DistanceToAngle.put(Preferences.getDouble("Hood_Map/DIST_MAP_FOUR", 5.1396),
                Preferences.getDouble("Hood_Map/ANGLE_MAP_FOUR", 0.0));
        DistanceToAngle.put(Preferences.getDouble("Hood_Map/DIST_MAP_FIVE", 6.0),
                Preferences.getDouble("Hood_Map/ANGLE_MAP_FIVE", 80.0));
    }

    public double getDistanceToAngle(double distance) {
        return DistanceToAngle.get(distance);
    }
}