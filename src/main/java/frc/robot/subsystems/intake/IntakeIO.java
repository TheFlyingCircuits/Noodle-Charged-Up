// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        double frontVelocityRPM = 0.0;
        double backVelocityRPM = 0.0;
        double frontAppliedVoltage = 0.0;
        double backAppliedVoltage = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    /**
     * Sets voltages of the intake wheels. Both sets of wheels will spin together.
     * @param frontVolts - The voltage to feed to the front motor. This will be POSITIVE to intake a cube, and negative to shoot a cube.
     * @param backVolts - The voltage to feed to the back motor. This will be POSITIVE to intake a cube, and negative to shoot a cube. 
     */
    public default void setIntakeWheelVoltages(double frontVolts, double backVolts) {}
}
