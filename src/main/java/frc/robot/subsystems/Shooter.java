package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final SparkFlex leftShooterMotor = new SparkFlex(ShootingConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    private final SparkFlex leftShooterMotor2 = new SparkFlex(ShootingConstants.LEFT_SHOOTER_MOTOR_ID_2, MotorType.kBrushless);
    private final SparkFlex rightShooterMotor = new SparkFlex(ShootingConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    private PIDController ShooterPID = new PIDController(ShootingConstants.DEFAULT_KP, 0, ShootingConstants.DEFAULT_KD);
    private SimpleMotorFeedforward ShooterFF = new SimpleMotorFeedforward(ShootingConstants.DEFAULT_KS, ShootingConstants.DEFAULT_KV);


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
//        Preferences.removeAll();

        initializePreferences();
        SparkFlexConfig leftShooterMotorConfig = new SparkFlexConfig();
        leftShooterMotorConfig.inverted(true);
        leftShooterMotorConfig.idleMode(IdleMode.kCoast);


        leftShooterMotorConfig.smartCurrentLimit(40);

        SparkFlexConfig rightShooterMotorConfig = new SparkFlexConfig();
        rightShooterMotorConfig.inverted(false);
        rightShooterMotorConfig.idleMode(IdleMode.kCoast);

        rightShooterMotorConfig.smartCurrentLimit(40);

        leftShooterMotor.configure(leftShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooterMotor2.configure(leftShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooterMotor.configure(rightShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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


    public void updateHardwareConfigs() {
        ShooterPID.setP(Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP));
        ShooterPID.setI(Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI));
        ShooterPID.setD(Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD));
        ShooterFF.setKv(Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV));
        ShooterFF.setKs(Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS));
    }

    public void setMotorRPM(double rpm) {
        double rps = rpm / 60.0;
        double PIDCalc = ShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity(), rpm);
        double FFcalc = ShooterFF.calculate(rpm);
        leftShooterMotor.setVoltage(PIDCalc + FFcalc);
        leftShooterMotor2.setVoltage(PIDCalc + FFcalc);
         rightShooterMotor.setVoltage(PIDCalc + FFcalc);
    }

    public void setMotorRPMBangBang(double rpm) {
        double bangOutput = shooterBangController.calculate(leftShooterMotor.getEncoder().getVelocity(), rpm);
        leftShooterMotor.setVoltage(bangOutput * 12.0);
        leftShooterMotor2.setVoltage(bangOutput * 12.0);
         rightShooterMotor.setVoltage(bangOutput * 12.0);
    }

    public boolean shooterAtSetpoint() {
        return filteredError < (ShootingConstants.SHOOTER_TOLERANCE_RPM / 60);
    }

    @Override
    public void periodic() {

        filteredError = errorFilter.calculate(
                Math.abs(ShooterPID.getError())
        );
        Logger.recordOutput("Shooter/FFOutput", ShooterFF.calculate(Preferences.getDouble("Shooter/RPM_SETPOINT", 0.0)));

        Logger.recordOutput("Shooter/LEFT_MOTOR_RPM", leftShooterMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/RIGHT_MOTOR_RPM", rightShooterMotor.getEncoder().getVelocity());
    }

    public void stopMotors() {
        leftShooterMotor.setVoltage(0.0);
        leftShooterMotor2.setVoltage(0.0);
         rightShooterMotor.setVoltage(0.0);
    }

    public void setLeftShooterMotor(double rpm) {
        leftShooterMotor.setVoltage(ShooterPID.calculate(leftShooterMotor.getEncoder().getPosition(), rpm));
    }

    public void setRightShooterMotor(double rpm) {
         rightShooterMotor.setVoltage(ShooterPID.calculate(rightShooterMotor.getEncoder().getPosition(), rpm));
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


    public double getDistanceToRPMM(double distance) {return DistanceToRPMLeft.get(distance);}
    public double getDistanceToShotTime(double distance) {return DistanceToShotTimeLeft.get(distance);}

}