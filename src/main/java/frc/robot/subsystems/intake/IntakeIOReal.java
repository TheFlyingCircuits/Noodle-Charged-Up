// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

    public CANSparkMax frontIntakeMotor;
    public CANSparkMax backIntakeMotor;
    public RelativeEncoder frontIntakeEncoder;
    public RelativeEncoder backIntakeEncoder;

    public IntakeIOReal() {
        System.out.println("[Init] Creating " + this.getClass().getName());
        frontIntakeMotor = new CANSparkMax(21, MotorType.kBrushless);
        backIntakeMotor = new CANSparkMax(13, MotorType.kBrushless);
        frontIntakeEncoder = frontIntakeMotor.getEncoder();
        backIntakeEncoder = frontIntakeMotor.getEncoder();
        configureMotors();
    }
    
    private void configureMotors() {
        frontIntakeMotor.setSmartCurrentLimit(50);
        backIntakeMotor.setSmartCurrentLimit(50);
        frontIntakeMotor.setInverted(true);
        backIntakeMotor.setInverted(true);
        frontIntakeMotor.setIdleMode(IdleMode.kBrake);
        backIntakeMotor.setIdleMode(IdleMode.kBrake);
        frontIntakeMotor.setOpenLoopRampRate(0.01);
        backIntakeMotor.setOpenLoopRampRate(0.01);
        frontIntakeMotor.burnFlash();
        backIntakeMotor.burnFlash();
    }

    @Override
    public void setIntakeWheelAmpLimits(int frontAmpLimit, int backAmpLimit) {
        frontIntakeMotor.setSmartCurrentLimit(frontAmpLimit);
        backIntakeMotor.setSmartCurrentLimit(backAmpLimit);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.frontVelocityRPM = frontIntakeEncoder.getVelocity();
        inputs.backVelocityRPM = backIntakeEncoder.getVelocity();
        inputs.frontAppliedVoltage = frontIntakeMotor.getAppliedOutput();
        inputs.backAppliedVoltage = backIntakeMotor.getAppliedOutput();
    }

    @Override
    public void setIntakeWheelVoltages(double frontVolts, double backVolts) {
        frontIntakeMotor.setVoltage(frontVolts);
        backIntakeMotor.setVoltage(backVolts);
    }

}
