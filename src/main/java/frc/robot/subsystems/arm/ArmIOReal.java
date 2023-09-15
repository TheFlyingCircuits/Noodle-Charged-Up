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

  public ArmIOReal() {
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
  }


  private void configMotors() {
    mLeftPivotMotor.restoreFactoryDefaults();
    mRightPivotMotor.restoreFactoryDefaults();
  
    //converts from rotations of the motor to radians of the arm
    mLeftPivotEncoder.setPositionConversionFactor((2*Math.PI)/Constants.Arm.gearReduction);
    mRightPivotEncoder.setPositionConversionFactor((2*Math.PI)/Constants.Arm.gearReduction);

    //converts from rpm of the motor to deg/s of the arm
    mLeftPivotEncoder.setVelocityConversionFactor((1./Constants.Arm.gearReduction) * 2*Math.PI / 60.0);
    mRightPivotEncoder.setVelocityConversionFactor((1./Constants.Arm.gearReduction) * 2*Math.PI / 60.0);


    mLeftPivotEncoder.setPosition(Constants.Arm.minAngleRadians);
    mRightPivotEncoder.setPosition(Constants.Arm.minAngleRadians);

    mLeftPivotMotor.setSmartCurrentLimit(30);
    mRightPivotMotor.setSmartCurrentLimit(30);
    mLeftPivotMotor.setInverted(Constants.Arm.leftPivotMotorInverted);
    mRightPivotMotor.setInverted(Constants.Arm.rightPivotMotorInverted);
    mLeftPivotMotor.setIdleMode(Constants.Arm.pivotIdleMode);
    mRightPivotMotor.setIdleMode(Constants.Arm.pivotIdleMode);
    mLeftPivotMotor.burnFlash();
    mRightPivotMotor.burnFlash();
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
        mLeftPivotEncoder.getPosition());
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
    inputs.atBackLimitSwitch = inputs.leftMotorPositionRadians >= Constants.Arm.maxAngleRadians; //backLimitSwitch.get();
  }

}
