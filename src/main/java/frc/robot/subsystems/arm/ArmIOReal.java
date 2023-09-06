// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
  private Rotation2d angleOffset;
  private CANSparkMax mLeftPivotMotor;
  private CANSparkMax mRightPivotMotor;
  private RelativeEncoder mLeftPivotEncoder;
  private RelativeEncoder mRightPivotEncoder;
  private CANCoder absoluteEncoder;
  private DigitalInput frontLimitSwitch;
  private DigitalInput backLimitSwitch;

  private double setpointRadians;

  private SparkMaxPIDController armPID;

  public ArmIOReal(int leftPivotMotorID, int rightPivotMotorID, int cancoderID) {
    System.out.println("[Init] Creating " + this.getClass().getName());

    // Limit Switches
    DigitalInput frontLimitSwitch = new DigitalInput(1);
    DigitalInput backLimitSwitch = new DigitalInput(2);

    // CANCoder Stuff
    angleOffset = new Rotation2d(Math.toRadians(Constants.Arm.encoderOffsetDegrees));
    absoluteEncoder = new CANCoder(cancoderID);

    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configMagnetOffset(angleOffset.getDegrees());
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    // Motor Stuff
    mLeftPivotMotor = new CANSparkMax(Constants.Arm.leftPivotMotorID, MotorType.kBrushless);
    mRightPivotMotor = new CANSparkMax(Constants.Arm.rightPivotMotorID, MotorType.kBrushless);

    // Encoders
    mLeftPivotEncoder = mLeftPivotMotor.getEncoder();
    mRightPivotEncoder = mRightPivotMotor.getEncoder();

    mLeftPivotEncoder.setPositionConversionFactor(Constants.Arm.gearReduction * 360.0);
    mRightPivotEncoder.setPositionConversionFactor(Constants.Arm.gearReduction * 360.0);

    syncRelativeEncodersAndCANCoder();

    // Sparks
    mLeftPivotMotor.restoreFactoryDefaults();
    mRightPivotMotor.restoreFactoryDefaults();

    mLeftPivotMotor.setSmartCurrentLimit(30);
    mRightPivotMotor.setSmartCurrentLimit(30);

    mLeftPivotMotor.setInverted(Constants.Arm.leftPivotMotorInverted);
    mRightPivotMotor.setInverted(Constants.Arm.rightPivotMotorInverted);

    mLeftPivotMotor.setIdleMode(Constants.Arm.pivotIdleMode);
    mRightPivotMotor.setIdleMode(Constants.Arm.pivotIdleMode);

    mLeftPivotMotor.burnFlash();
    mRightPivotMotor.burnFlash();

    mRightPivotMotor.follow(mLeftPivotMotor, true);
    armPID = mLeftPivotMotor.getPIDController();

    armPID.setP(Constants.Arm.Kp);
    armPID.setI(Constants.Arm.Ki);
    armPID.setD(Constants.Arm.Kd);

    armPID.setFF(Constants.Arm.Kf);

  }

  public void setArmVoltage(double volts) {
    mLeftPivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    mRightPivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setArmVoltage(0.0);
    }

    inputs.armPosition = new Rotation2d(
        Units.degreesToRadians(mLeftPivotEncoder.getPosition()));
    inputs.armRotateRadiansPerSecond = Units.degreesToRadians(mLeftPivotEncoder.getVelocity());
    inputs.leftPivotMotorArmVolts = mLeftPivotMotor.getAppliedOutput();
    inputs.rightPivotMotorArmVolts = mRightPivotMotor.getAppliedOutput();
    inputs.leftPivotMotorArmCurrentAmps = mLeftPivotMotor.getOutputCurrent();
    inputs.rightPivotMotorArmCurrentAmps = mRightPivotMotor.getOutputCurrent();
    inputs.leftPivotMotorTempCelsius = mLeftPivotMotor.getMotorTemperature();
    inputs.rightPivotMotorTempCelsius = mRightPivotMotor.getMotorTemperature();
    inputs.atFrontLimitSwitch = frontLimitSwitch.get();
    inputs.atBackLimitSwitch = backLimitSwitch.get();
  }

  public void syncRelativeEncodersAndCANCoder() {
    mLeftPivotEncoder.setPosition(absoluteEncoder.getAbsolutePosition());
    mRightPivotEncoder.setPosition(absoluteEncoder.getAbsolutePosition());
  }

  /**
   * Set the position setpoint of the arm in radians.
   */
  public void setArmSetpointRadians(double radians) {
    // Check that setpoint is in allowable range
    double setpoint = MathUtil.clamp(radians, Constants.Arm.minAngleDegrees, Constants.Arm.maxAngleDegrees);

    if(!frontLimitSwitch.get() && !backLimitSwitch.get()) {
      armPID.setReference(setpoint, ControlType.kPosition);
    }
  }

}
