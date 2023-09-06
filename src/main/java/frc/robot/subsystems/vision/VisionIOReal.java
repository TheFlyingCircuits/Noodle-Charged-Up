// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Vision.LimelightTarget;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {

  private static NetworkTable limelight;

  public VisionIOReal() {

    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.tx = limelight.getEntry("tx").getDouble(0.0);
    inputs.ty = limelight.getEntry("ty").getDouble(0.0);

    double[] defaultValues = { 0, 0, 0, 0, 0, 0 };
    inputs.botpose_targetspace = limelight.getEntry("botpose_targetspace").getDoubleArray(defaultValues);
    inputs.botpose_wpiblue = limelight.getEntry("botpose_wpiblue").getDoubleArray(defaultValues);

    inputs.tl = limelight.getEntry("tl").getDouble(0.0);
    inputs.cl = limelight.getEntry("cl").getDouble(0.0);

    inputs.ta = limelight.getEntry("ta").getDouble(0.0);
  }

  @Override
  public void changePipeline(LimelightTarget target) {
    limelight.getEntry("pipeline").setNumber(target.limelightPipeline);
  }
}