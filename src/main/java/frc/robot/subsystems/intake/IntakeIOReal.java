// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

    public CANSparkMax frontIntakeMotor;
    public CANSparkMax backIntakeMotor;

    public IntakeIOReal() {
        frontIntakeMotor = new CANSparkMax(15, MotorType.kBrushless);
        backIntakeMotor = new CANSparkMax(21, MotorType.kBrushless);
        configureMotors();
    }
    
    private void configureMotors() {
        frontIntakeMotor.setSmartCurrentLimit(15);
        backIntakeMotor.setSmartCurrentLimit(15);
        frontIntakeMotor.setInverted(true);
        backIntakeMotor.setInverted(true);
        frontIntakeMotor.setIdleMode(IdleMode.kBrake);
        backIntakeMotor.setIdleMode(IdleMode.kBrake);
        frontIntakeMotor.burnFlash();
        backIntakeMotor.burnFlash();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.frontVelocityRPM = 0.0;
        inputs.backVelocityRPM = 0.0;
        inputs.frontAppliedVoltage = frontIntakeMotor.getAppliedOutput();
        inputs.backAppliedVoltage = backIntakeMotor.getAppliedOutput();
    }

    @Override
    public void setIntakeWheelVoltages(double frontVolts, double backVolts) {
        frontIntakeMotor.setVoltage(frontVolts);
        backIntakeMotor.setVoltage(backVolts);
    }

}
