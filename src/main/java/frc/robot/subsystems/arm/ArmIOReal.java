// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
  private CANSparkMax mLeftPivotMotor;
  private CANSparkMax mRightPivotMotor;
  private RelativeEncoder mLeftPivotEncoder;
  private RelativeEncoder mRightPivotEncoder;
  private DigitalInput frontLimitSwitch;
  private DigitalInput backLimitSwitch;

  public ArmIOReal(int leftPivotMotorID, int rightPivotMotorID, int cancoderID) {
    System.out.println("[Init] Creating " + this.getClass().getName());

    // Limit Switches
    frontLimitSwitch = new DigitalInput(1);
    backLimitSwitch = new DigitalInput(2);

    // Motor Stuff
    mLeftPivotMotor = new CANSparkMax(Constants.Arm.leftPivotMotorID, MotorType.kBrushless);
    mRightPivotMotor = new CANSparkMax(Constants.Arm.rightPivotMotorID, MotorType.kBrushless);

    // Encoders
    mLeftPivotEncoder = mLeftPivotMotor.getEncoder();
    mRightPivotEncoder = mRightPivotMotor.getEncoder();




    configMotors();
    configEncoders();
  }

  private void configEncoders() {
    //TODO: FIND GEAR REDUCTIONS

    //converts from rotation sof the motor to degrees of the arm
    mLeftPivotEncoder.setPositionConversionFactor(Constants.Arm.gearReduction * 360.0);
    mRightPivotEncoder.setPositionConversionFactor(Constants.Arm.gearReduction * 360.0);

    //converts from rpm of the motor to deg/s of the arm
    mLeftPivotEncoder.setVelocityConversionFactor(Constants.Arm.gearReduction * 360.0 / 60.0);
    mRightPivotEncoder.setVelocityConversionFactor(Constants.Arm.gearReduction * 360.0 / 60.0);

  }

  private void configMotors() {
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
  }

  @Override
  public void setArmVoltage(double volts) {
    mLeftPivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    mRightPivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setArmVoltage(0.0);
    }

    inputs.armPosition = new Rotation2d(
        Units.degreesToRadians(mLeftPivotEncoder.getPosition()));
    inputs.armRotateDegreesPerSecond = mLeftPivotEncoder.getVelocity();
    inputs.leftPivotMotorArmVolts = mLeftPivotMotor.getAppliedOutput();
    inputs.rightPivotMotorArmVolts = mRightPivotMotor.getAppliedOutput();
    inputs.leftPivotMotorArmCurrentAmps = mLeftPivotMotor.getOutputCurrent();
    inputs.rightPivotMotorArmCurrentAmps = mRightPivotMotor.getOutputCurrent();
    inputs.leftPivotMotorTempCelsius = mLeftPivotMotor.getMotorTemperature();
    inputs.rightPivotMotorTempCelsius = mRightPivotMotor.getMotorTemperature();
    inputs.atFrontLimitSwitch = frontLimitSwitch.get();
    inputs.atBackLimitSwitch = backLimitSwitch.get();
  }

}
