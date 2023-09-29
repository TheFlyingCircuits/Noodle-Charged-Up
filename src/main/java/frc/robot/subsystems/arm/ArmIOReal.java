// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
  private CANSparkMax mLeftPivotMotor;
  private CANSparkMax mRightPivotMotor;
  private RelativeEncoder mLeftPivotEncoder;
  private RelativeEncoder mRightPivotEncoder;
  // private DigitalInput frontLimitSwitch;
  private DigitalInput backLimitSwitch;

  public ArmIOReal() {
    System.out.println("[Init] Creating " + this.getClass().getName());

    // Motor Stuff
    mLeftPivotMotor = new CANSparkMax(Constants.Arm.leftPivotMotorID, MotorType.kBrushless);
    mRightPivotMotor = new CANSparkMax(Constants.Arm.rightPivotMotorID, MotorType.kBrushless);

    // Encoders
    mLeftPivotEncoder = mLeftPivotMotor.getEncoder();
    mRightPivotEncoder = mRightPivotMotor.getEncoder();

    //limit switches
    backLimitSwitch = new DigitalInput(0);

    configMotors();
  }


  private void configMotors() {
    mLeftPivotMotor.restoreFactoryDefaults();
    mRightPivotMotor.restoreFactoryDefaults();

    
    mLeftPivotMotor.setInverted(Constants.Arm.leftPivotMotorInverted);
    mRightPivotMotor.setInverted(Constants.Arm.rightPivotMotorInverted);
  
    //converts from rotations of the motor to radians of the arm
    mLeftPivotEncoder.setPositionConversionFactor((2*Math.PI)/Constants.Arm.gearReduction);
    mRightPivotEncoder.setPositionConversionFactor((2*Math.PI)/Constants.Arm.gearReduction);

    //converts from rpm of the motor to deg/s of the arm
    mLeftPivotEncoder.setVelocityConversionFactor((1./Constants.Arm.gearReduction) * 2*Math.PI / 60.0);
    mRightPivotEncoder.setVelocityConversionFactor((1./Constants.Arm.gearReduction) * 2*Math.PI / 60.0);




    mLeftPivotMotor.setSmartCurrentLimit(20);
    mRightPivotMotor.setSmartCurrentLimit(20);

    mLeftPivotMotor.setIdleMode(Constants.Arm.pivotIdleMode);
    mRightPivotMotor.setIdleMode(Constants.Arm.pivotIdleMode);

    mLeftPivotEncoder.setPosition(Constants.Arm.startingAngleRadians);
    mRightPivotEncoder.setPosition(Constants.Arm.startingAngleRadians);

    mLeftPivotMotor.burnFlash();
    mRightPivotMotor.burnFlash();
  }

  @Override
  public void setArmVoltage(double volts) {
    mLeftPivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    mRightPivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setArmPosition(double positionRadians) {
    mLeftPivotEncoder.setPosition(positionRadians);
    mRightPivotEncoder.setPosition(positionRadians);
  }

  public void setToBreakMode(boolean isInBreak) {
    if(isInBreak) {
      mLeftPivotMotor.setIdleMode(IdleMode.kBrake);
      mRightPivotMotor.setIdleMode(IdleMode.kBrake);
    }
    else {
      mLeftPivotMotor.setIdleMode(IdleMode.kCoast);
      mRightPivotMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setArmVoltage(0.0);
    }

    inputs.armPosition = new Rotation2d(
        mRightPivotEncoder.getPosition());
    inputs.leftMotorPositionRadians = mLeftPivotEncoder.getPosition();
    inputs.rightMotorPositionRadians = mRightPivotEncoder.getPosition();
    inputs.armVelocityRadiansPerSecond = mLeftPivotEncoder.getVelocity();
    inputs.leftPivotMotorArmVolts = mLeftPivotMotor.getAppliedOutput();
    inputs.rightPivotMotorArmVolts = mRightPivotMotor.getAppliedOutput();
    inputs.leftPivotMotorArmCurrentAmps = mLeftPivotMotor.getOutputCurrent();
    inputs.rightPivotMotorArmCurrentAmps = mRightPivotMotor.getOutputCurrent();
    inputs.leftPivotMotorTempCelsius = mLeftPivotMotor.getMotorTemperature();
    inputs.rightPivotMotorTempCelsius = mRightPivotMotor.getMotorTemperature();
    
    //TODO: REPLACE THESE WHEN LIMIT SWITCHES ARE INSTALLED
    inputs.atFrontLimitSwitch = inputs.leftMotorPositionRadians <= Constants.Arm.minAngleRadians;//frontLimitSwitch.get();
    inputs.atBackLimitSwitch = backLimitSwitch.get();
  }

}
