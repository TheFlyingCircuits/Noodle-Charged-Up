// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModuleIOReal implements SwerveModuleIO {
    private double angleOffset;
    private CANCoder absoluteEncoder;
    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private RelativeEncoder mDriveEncoder;
    private RelativeEncoder mAngleEncoder;

    public SwerveModuleIOReal(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
        this.angleOffset=angleOffset;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANCoder(cancoderID);
        configCANCoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        mAngleEncoder = mAngleMotor.getEncoder();
        configAngleMotor();

        // This encoder will always have units of degrees of the wheel.
        mAngleEncoder.setPosition(absoluteEncoder.getAbsolutePosition());

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mDriveEncoder = mDriveMotor.getEncoder();
        configDriveMotor();
    }

    //TODO: i hope this is correct usage of @override? simon pls check
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = mDriveEncoder.getPosition();
        inputs.driveVelocityMetersPerSecond = mDriveEncoder.getVelocity();
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition();
    }

    @Override
    public void setDriveVoltage(double volts) {
        mDriveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        mAngleMotor.setVoltage(volts);
    }

    private void configCANCoder() {
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        absoluteEncoder.configMagnetOffset(angleOffset);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        //converts rotations of motor into deg of wheel
        mAngleEncoder.setPositionConversionFactor(Constants.Swerve.steerReduction*360.0);
        //converts rpm of motor into deg/s of wheel
        mAngleEncoder.setVelocityConversionFactor(Constants.Swerve.steerReduction*360.0/60.0);
        mAngleMotor.burnFlash();
    }

    private void configDriveMotor() {
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        
        mDriveEncoder.setPosition(0.0);

        // Converts rotations of motor to meters traveled of wheel.
        mDriveEncoder.setPositionConversionFactor(
            Constants.Swerve.driveReduction
            * Constants.Swerve.wheelCircumferenceMeters
        );

        // Converts rpm of motor to m/s of wheel.
        mDriveEncoder.setVelocityConversionFactor(
            1./60. * Constants.Swerve.driveReduction
            * Constants.Swerve.wheelCircumferenceMeters
        );

        REVLibError err = mDriveEncoder.setMeasurementPeriod(10);
        if (err == REVLibError.kOk) {
            System.out.println("successfully set drive encoder measurement period");
        }
        
        err = mDriveEncoder.setAverageDepth(2);
        if (err == REVLibError.kOk) {
            System.out.println("successfully set drive encoder window size");
        }

        mDriveMotor.burnFlash();
    }
}