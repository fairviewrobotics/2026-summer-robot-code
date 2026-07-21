package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final TalonFX leftShooterMotor = new TalonFX(ShootingConstants.LEFT_SHOOTER_MOTOR_ID);
    private final TalonFX leftShooterMotor2 = new TalonFX(ShootingConstants.LEFT_SHOOTER_MOTOR_ID);
    private final TalonFX rightShooterMotor = new TalonFX(ShootingConstants.RIGHT_SHOOTER_MOTOR_ID);
    private final TalonFX rightShooterMotor2 = new TalonFX(ShootingConstants.RIGHT_SHOOTER_MOTOR_ID);

    private double lastKP = ShootingConstants.DEFAULT_KP;
    private double lastKI = ShootingConstants.DEFAULT_KI;
    private double lastKD = ShootingConstants.DEFAULT_KD;
    private double lastKV = ShootingConstants.DEFAULT_KV;
    private double lastKS = ShootingConstants.DEFAULT_KS;
    private double lastRPM = ShootingConstants.SHOOTER_RPM;
    private double lastMap1 = 2000.0;
    private double lastMap2 = 2350.0;
    private double lastMap3 = 3000.0;
    private double lastMap4 = 3450.0;
    private double lastMap5 = 4500.0;

    private final InterpolatingDoubleTreeMap DistanceToRPMLeft =
            new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap DistanceToShotTimeLeft =
            new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap DistanceToRPMRight =
            new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap DistanceToShotTimeRight =
            new InterpolatingDoubleTreeMap();

    private final LinearFilter errorFilter = LinearFilter.movingAverage(5);
    private double filteredError = 0;

    private final BangBangController shooterBangController = new BangBangController();

    public Shooter() {

        // Get rid of this at some point, Daniel
        Preferences.removeAll();

        initializePreferences();
        TalonFXConfiguration leftShooterMotorConfig = new TalonFXConfiguration();
        leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftShooterMotorConfig.Slot0.kP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        leftShooterMotorConfig.Slot0.kI = Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        leftShooterMotorConfig.Slot0.kD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        leftShooterMotorConfig.Slot0.kV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        leftShooterMotorConfig.Slot0.kS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);

        leftShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        leftShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;

        leftShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftShooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;

        leftShooterMotorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        leftShooterMotorConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        TalonFXConfiguration rightShooterMotorConfig = new TalonFXConfiguration();
        rightShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightShooterMotorConfig.Slot0.kP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        rightShooterMotorConfig.Slot0.kD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        rightShooterMotorConfig.Slot0.kV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        rightShooterMotorConfig.Slot0.kS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);

        rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        rightShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;

        rightShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightShooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;

        rightShooterMotorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        rightShooterMotorConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
        leftShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
        rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
        rightShooterMotor2.getConfigurator().apply(rightShooterMotorConfig);

        createDistanceToRPMMap();
        createDistanceToShotTimeMap();
    }

    private void initializePreferences() {
        Preferences.initDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        Preferences.initDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        Preferences.initDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        Preferences.initDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        Preferences.initDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);
        Preferences.initDouble("Shooter/RPM_SETPOINT", ShootingConstants.SHOOTER_RPM);
        Preferences.initDouble("Shooter_Map/RPM_MAP_ONE", 2000.0);
        Preferences.initDouble("Shooter_Map/RPM_MAP_TWO", 2350.0);
        Preferences.initDouble("Shooter_Map/RPM_MAP_THREE", 3000.0);
        Preferences.initDouble("Shooter_Map/RPM_MAP_FOUR", 3450.0);
        Preferences.initDouble("Shooter_Map/RPM_MAP_FIVE", 4500.0);
    }


    private void updateHardwareConfigs() {
        var slot0Config = new TalonFXConfiguration().Slot0;
        slot0Config.kP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        slot0Config.kI = Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        slot0Config.kD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        slot0Config.kV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        slot0Config.kS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);

        leftShooterMotor.getConfigurator().apply(slot0Config);
        leftShooterMotor2.getConfigurator().apply(slot0Config);
        rightShooterMotor.getConfigurator().apply(slot0Config);
        rightShooterMotor2.getConfigurator().apply(slot0Config);

    }

    public void setMotorRPM(double rpm) {
        double rps = rpm / 60.0;
        leftShooterMotor.setControl(new VelocityVoltage(rps));
        leftShooterMotor2.setControl(new VelocityVoltage(rps));
        rightShooterMotor.setControl(new VelocityVoltage(rps));
        rightShooterMotor2.setControl(new VelocityVoltage(rps));
    }

    public void setMotorRPMBangBang(double rpm) {
        double bangOutput = shooterBangController.calculate(leftShooterMotor.getVelocity().getValueAsDouble() * 60, rpm);
        leftShooterMotor.setVoltage(bangOutput * 12.0);
        leftShooterMotor2.setVoltage(bangOutput * 12.0);
        rightShooterMotor.setVoltage(bangOutput * 12.0);
        rightShooterMotor2.setVoltage(bangOutput * 12.0);
    }

    public boolean shooterAtSetpoint() {
        return filteredError < (ShootingConstants.SHOOTER_TOLERANCE_RPM / 60);
    }

    @Override
    public void periodic() {

        filteredError = errorFilter.calculate(
                Math.abs(leftShooterMotor.getClosedLoopError().getValueAsDouble())
        );

        Logger.recordOutput("Shooter/LEFT_MOTOR_RPM", leftShooterMotor.getVelocity().getValueAsDouble() * 60);
        Logger.recordOutput("Shooter/RIGHT_MOTOR_RPM", rightShooterMotor.getVelocity().getValueAsDouble() * 60);
    }

    public void stopMotors() {
        leftShooterMotor.stopMotor();
        leftShooterMotor2.stopMotor();
        rightShooterMotor.stopMotor();
        rightShooterMotor2.stopMotor();
    }

    public void setLeftShooterMotor(double rpm) {
        leftShooterMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public void setRightShooterMotor(double rpm) {
        rightShooterMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public void setLeftShooterMotorVoltage(double voltage) {
        leftShooterMotor.setVoltage(voltage);
    }

    public void setRightShooterMotorVoltage(double voltage) { rightShooterMotor.setVoltage(voltage); }

    public void setBothMotorsPreferences() {
        setLeftShooterMotor(Preferences.getDouble("Shooter/RPM_SETPOINT", ShootingConstants.SHOOTER_RPM));
        setRightShooterMotor(Preferences.getDouble("Shooter/RPM_SETPOINT", ShootingConstants.SHOOTER_RPM));
    }

    private void createDistanceToRPMMap() {
        DistanceToRPMLeft.put(0.0, 2000.0);
        DistanceToRPMLeft.put(3.0796, 2350.0);
        DistanceToRPMLeft.put(4.1596, 3000.0);
        DistanceToRPMLeft.put(5.1396, 3450.0);
        DistanceToRPMLeft.put(6.0, 4500.0);
    }

    private void createDistanceToShotTimeMap() {
        DistanceToShotTimeLeft.put(0.0, 0.3);
        DistanceToShotTimeLeft.put(3.0796, 0.785);
        DistanceToShotTimeLeft.put(4.1596, 0.995);
        DistanceToShotTimeLeft.put(5.1396, 1.265);
        DistanceToShotTimeLeft.put(8.0, 2.0);
    }


    public double getDistanceToRPMMapLeft(double distance) {return DistanceToRPMLeft.get(distance);}
    public double getDistanceToShotTimeLeft(double distance) {return DistanceToShotTimeLeft.get(distance);}
    public double getDistanceToRPMMapRight(double distance) {return DistanceToRPMRight.get(distance);}
    public double getDistanceToShotTimeRight(double distance) {return DistanceToShotTimeRight.get(distance);}

}