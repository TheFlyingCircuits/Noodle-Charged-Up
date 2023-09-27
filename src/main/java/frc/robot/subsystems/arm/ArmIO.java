// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        Rotation2d armPosition = new Rotation2d();
        double leftMotorPositionRadians = 0.0;
        double rightMotorPositionRadians = 0.0;
        double armVelocityRadiansPerSecond = 0.0;
        double leftPivotMotorArmVolts = 0.0;
        double rightPivotMotorArmVolts = 0.0;
        double leftPivotMotorArmCurrentAmps = 0.0;
        double rightPivotMotorArmCurrentAmps = 0.0;
        double leftPivotMotorTempCelsius = 0.0;
        double rightPivotMotorTempCelsius = 0.0;
        boolean atFrontLimitSwitch = false;
        boolean atBackLimitSwitch = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {};

    /** Run the arm pivot motors at the specified voltage
     * @param volts voltage to run motors at. Positive moves the arm "forward" towards front of robot
     */
    public default void setArmVoltage(double volts) {}

    public default void setArmPosition(double positionRadians) {}


}
