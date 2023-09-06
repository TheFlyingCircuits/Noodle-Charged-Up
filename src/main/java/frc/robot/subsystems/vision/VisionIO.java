// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.Vision.LimelightTarget;

/** Add your docs here. */
public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public double tx = 0.0;
        public double ty = 0.0;

        public double[] botpose_targetspace = {0, 0, 0, 0, 0, 0};
        public double[] botpose_wpiblue = {0, 0, 0, 0, 0, 0};

        public double tl = 0.0;
        public double cl = 0.0;

        public double ta = 0.0;
    }

    public default void updateInputs(VisionIOInputs inputs) {};

    public default void changePipeline(LimelightTarget target) {};
}