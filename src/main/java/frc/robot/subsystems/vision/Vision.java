// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */


  private VisionIO io;
  private static VisionIOInputsAutoLogged inputs;

  private Pose2d meanPoseMeasurement;
  private double xVarianceMetersSquared;
  private double yVarianceMetersSquared;
  private double rVarianceRadiansSquared;
  private int iteration = 1;

  public Vision(VisionIO io) {

    this.io = io;
    inputs = new VisionIOInputsAutoLogged();

    xVarianceMetersSquared = 0;
    yVarianceMetersSquared = 0;
    rVarianceRadiansSquared = 0;
  }

  public void changePipeline(Constants.Vision.LimelightTarget target) {
    //////////////////////////////
  }


  public double getTargetDistanceMeters(Constants.Vision.LimelightTarget target) {

    return (target.heightMeters - Constants.Vision.limelightHeightMeters)
        / Math.tan(
            Math.toRadians(inputs.ty + Constants.Vision.limelightAngleDegrees));
  }

  public double getLateralOffsetMeters(Constants.Vision.LimelightTarget target) {
    return getTargetDistanceMeters(target) * Math.tan(Math.toRadians(inputs.tx));
  }


  public static double getLimelightAprilTagDistanceMeters() {
    
    //for some reason the first and third values here represent the x and y position of the robot relative to the target
    double[] limelightPoseArray = inputs.botpose_targetspace;

    return Math.sqrt(limelightPoseArray[0]*limelightPoseArray[0] + limelightPoseArray[1]*limelightPoseArray[1]);
  }

  //the next three methods are static because drivetrain references them
  //gets pose using limelight 3d tingies
  public static Pose2d getLimelightPose2d() {

    //limelight sends its pose estimate as an array of {x, y, z, roll, pitch, yaw} (in meters and degrees)
    double[] limelightPoseArray = inputs.botpose_wpiblue;

    return new Pose2d(
      limelightPoseArray[0],
      limelightPoseArray[1],
      Rotation2d.fromDegrees(limelightPoseArray[5]));
  }

  public static double getTotalLatencyMs() {

    return inputs.tl + inputs.cl;
  }

  //idk if this is the best way of implementing this but whatever
  public static boolean limelightSeesAprilTag() {
    //ta is the area on the camera of the target it sees
    if (inputs.ta != 0) return true;
    else return false;
  }


  //look at the stackexchange link below if you want some context for this ugliness
  //returns variance (units^2) of measurement, given repeated incremental measurements
  double calculateIterativeVariance(double curVariance, double iterations, double measurement, double prevMean) {
    return (iterations-2)/(iterations-1)*curVariance*curVariance + Math.pow(measurement-prevMean, 2)/iterations;
  }

  //call this every loop
  private void updateVisionPoseStdDev(double n) {


    //based on this: https://math.stackexchange.com/questions/102978/incremental-computation-of-standard-deviation

    if (meanPoseMeasurement == null) {
      meanPoseMeasurement = getLimelightPose2d();
      return;
    }

    Pose2d curPose = getLimelightPose2d();
    
    xVarianceMetersSquared = calculateIterativeVariance(xVarianceMetersSquared, n, curPose.getX(), meanPoseMeasurement.getX());
    yVarianceMetersSquared = calculateIterativeVariance(yVarianceMetersSquared, n, curPose.getY(), meanPoseMeasurement.getY());
    rVarianceRadiansSquared = calculateIterativeVariance(rVarianceRadiansSquared, n, curPose.getRotation().getRadians(), meanPoseMeasurement.getRotation().getRadians());

    meanPoseMeasurement = new Pose2d(
      (meanPoseMeasurement.getX()*(n-1) + curPose.getX())/n,
      (meanPoseMeasurement.getY()*(n-1) + curPose.getY())/n,
      new Rotation2d(
        (meanPoseMeasurement.getRotation().getRadians()*(n-1) + curPose.getRotation().getRadians())/n
      )
    );
  }
  

  public void resetVisionPoseStdDev() {
    iteration = 1;
    xVarianceMetersSquared = 0;
    yVarianceMetersSquared = 0;
    rVarianceRadiansSquared = 0;
    meanPoseMeasurement = null;
  }

  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    updateVisionPoseStdDev(iteration);
    iteration++;

    Logger.getInstance().recordOutput("vision/limelightPose2d", getLimelightPose2d());
  }
}