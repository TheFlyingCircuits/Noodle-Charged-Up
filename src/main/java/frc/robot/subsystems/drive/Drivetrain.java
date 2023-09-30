// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

public class Drivetrain extends SubsystemBase {

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs;
  private SwerveModule[] mSwerveMods;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveDrivePoseEstimator poseEstimator;

  private Field2d odometryField;
  private Field2d estimatorField;

  private SlewRateLimiter chassisSpeedsXSlewLimiter;
  private SlewRateLimiter chassisSpeedsYSlewLimiter;

  public Drivetrain(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO) {
    System.out.println("[Init] Creating " + this.getClass().getName());


    this.gyroIO = gyroIO;
    gyroInputs = new GyroIOInputsAutoLogged();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(flIO, 0),
        new SwerveModule(frIO, 1),
        new SwerveModule(blIO, 2),
        new SwerveModule(brIO, 3)
    };

    gyroIO.zeroYaw();
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRobotRotation2d(),
        getModulePositions());

    Matrix<N3, N1> stateStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
    // corresponds to x, y, and rotation standard deviations (meters and radians)
    stateStdDevs.set(0, 0, 0.1);
    stateStdDevs.set(1, 0, 0.1);
    stateStdDevs.set(2, 0, 0.005);
    Matrix<N3, N1> visionStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
    // corresponds to x, y, and rotation standard deviations (meters and radians)
    // these values are automatically recalculated periodically depending on
    // distance
    visionStdDevs.set(0, 0, 0.03);
    visionStdDevs.set(1, 0, 0.03);
    visionStdDevs.set(2, 0, 1.3);

    poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        getRobotRotation2d(),
        getModulePositions(),
        new Pose2d(0, 0, new Rotation2d(0)),
        stateStdDevs,
        visionStdDevs);

    odometryField = new Field2d();
    estimatorField = new Field2d();

    chassisSpeedsXSlewLimiter = new SlewRateLimiter(Constants.Swerve.maxDesiredDriverAccel);
    chassisSpeedsYSlewLimiter = new SlewRateLimiter(Constants.Swerve.maxDesiredDriverAccel);
  }

  // GYROSCOPE

  // The navX comments are bad don't trust them.

  /**
   * Sets the gyroscope angle to zero.
   * <br>
   * This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroYaw() {
    gyroIO.zeroYaw(); // this is the exact same thing as saying gyro.reset();
    gyroIO.setAngleAdjustment(0);

    setPoseMeters(
        new Pose2d(
            getPoseMeters().getTranslation(),
            Rotation2d.fromDegrees(0)));
  }


    /**
   * Sets the gyroscope angle adjustment to be 180.
   * <br>
   * In other words, if the robot is facing towards the grid, calling this method will field orient it.
   */
  public void setGyroscope180() {
    gyroIO.zeroYaw();
    gyroIO.setAngleAdjustment(180);

    swerveOdometry.resetPosition(getRobotRotation2d(), getModulePositions(), getPoseMeters());
  }

  /**
   * Zeros the robot gyro and then adds 180 degrees (or pi radians) to the robot's rotation pose
   * <br>
   * In other words, if the robot is facing towards the grid, calling this method
   * will field orient it.
   */
  public void setPose2D180() {
    gyroIO.zeroYaw();
    setPoseMeters(
      new Pose2d(
        getPoseMeters().getTranslation(),
        new Rotation2d(Math.PI)
      )
    );
  }

  // TODO: consider refactoring code to separate pose rotation and gyro rotation.
  /**
   * Gets the angle of the robot measured by the gyroscope as a Rotation2d
   * (continuous).
   * 
   * @return rotation2d - this angle will be counterclockwise positive.
   */
  public Rotation2d getRobotRotation2d() {
    return Rotation2d.fromDegrees(gyroInputs.robotYawDegrees);
  }

  /**
   * Returns the current pitch value (in degrees, from -180 to 180) of the robot,
   * based off of the NavX.
   * <br>
   * Pitch is a measure of angle between the robot-oriented X-axis and the
   * horizontal.
   * 
   * @return The current pitch value in degrees (-180 to 180). This value will be
   *         positive if the front of the robot is raised.
   */
  public double getRobotPitchDegrees() {
    return -gyroInputs.robotPitchDegrees;
  }

  /**
   * Returns the current roll value (in degrees, from -180 to 180) of the robot,
   * based off of the NavX.
   * Roll is a measure of angle between the robot-oriented Y-axis and the
   * horizontal.
   * 
   * @return The current roll value in degrees (-180 to 180). This value will be
   *         positive if the left of the robot (positive Y) is raised.
   */
  public double getRobotRollDegrees() {
    // Since navx is mounted silly and also axis convention this is correct.
    return -gyroInputs.robotRollDegrees;
  }

  /**
   * Returns the current angular pitch velocity in degrees per second of the
   * robot, based off of hte NavX.
   * This represents how quickly the angle between the robot-oriented X-axis and
   * the horizontal changes.
   * 
   * @return The current pitch speed value in degrees per second. If the front of
   *         the robot is being raised, this will return a positive value.
   */
  public double getRobotPitchDegreesPerSecond() {
    return gyroInputs.robotPitchDegreesPerSecond;
  }

  /**
   * Returns the current angular roll velocity in degrees per second of the robot,
   * based off of the NavX.
   * This represents how quickly the angle between the robot-oriented Y-axis and
   * the horizontal changes.
   * 
   * @return The current roll speed value in degrees per second. If the left side
   *         of the robot is being raised, this will return a positive value.
   */
  public double getRobotRollDegreesPerSecond() {
    return gyroInputs.robotRollDegreesPerSecond;
  }

  // POSE, FIELD, ODOMETRY

  /**
   * Gets the current position of the robot on the field in meters.
   * This value considers the origin to be the right side of the robot's current
   * alliance.
   * <p>
   * A positive X value brings the robot towards the opposing alliance,
   * and a positive Y value brings the robot left as viewed by your alliance.
   * 
   * @return The current position of the robot on the field in meters.
   */
  public Pose2d getPoseMeters() {
    return swerveOdometry.getPoseMeters();
  }

  public Pose2d getEstimatorPoseMeters() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Sets the current position of the robot on the field in meters.
   * <p>
   * A positive X value brings the robot towards the opposing alliance,
   * and a positive Y value brings the robot left as viewed by your alliance.
   * 
   * @param pose
   */
  public void setPoseMeters(Pose2d pose) {
    swerveOdometry.resetPosition(
        getRobotRotation2d(),
        getModulePositions(),
        pose);
    poseEstimator.resetPosition(
        getRobotRotation2d(),
        getModulePositions(),
        pose);
  }

  public void updateOdometry() {
    swerveOdometry.update(getRobotRotation2d(), getModulePositions());

    // updates field visualization on shuffleboard
    odometryField.setRobotPose(swerveOdometry.getPoseMeters());
  }

  // updates pose estimator with both vision and odometry measurement
  private void updatePoseEstimator() {
    poseEstimator.update(
        getRobotRotation2d(), getModulePositions());

    // calculate inaccuracy of limelight source by finding distance between sources
    double limelightOdometryDistance = Math.sqrt(
        Math.pow(Vision.getLimelightPose2d().getX() - getPoseMeters().getX(), 2)
            + Math.pow(Vision.getLimelightPose2d().getY() - getPoseMeters().getY(), 2));

    // scales standard deviations from lower value to upper value from distance 0 to
    // 2.4 meters away from target
    double xyStdDevMeters = MathUtil.interpolate(0.005, 0.4, Vision.getLimelightAprilTagDistanceMeters() / 2.4);
    double rStdDevRadians = MathUtil.interpolate(0.01, 1.0, Vision.getLimelightAprilTagDistanceMeters() / 2.4);

    Matrix<N3, N1> stdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
    // corresponds to x, y, and rotation standard deviations (meters and radians)
    stdDevs.set(0, 0, xyStdDevMeters);
    stdDevs.set(1, 0, xyStdDevMeters);
    stdDevs.set(2, 0, rStdDevRadians);

    if (Vision.limelightSeesAprilTag() && (limelightOdometryDistance < 0.3)
        && Vision.getLimelightAprilTagDistanceMeters() < 2.4) {
      poseEstimator.addVisionMeasurement(
          Vision.getLimelightPose2d(),
          Timer.getFPGATimestamp() - (Vision.getTotalLatencyMs() / 1000.0),
          stdDevs);
    }

    // updates field visualization on shuffleboard
    estimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  // resets odometry pose to the pose given by limelight
  public void setPoseToVisionEstimate() {
    setPoseMeters(Vision.getLimelightPose2d());
  }

  // SWERVE MODULES

  /**
   * Drives the robot based on a desired ChassisSpeeds.
   * <p>
   * Takes in a robot relative ChassisSpeeds. Field relative control can be
   * accomplished by using the ChassisSpeeds.fromFieldRelative() method.
   * 
   * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters
   *                             per second and radians per second.
   */
  public void drive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {

    desiredChassisSpeeds.vxMetersPerSecond = chassisSpeedsXSlewLimiter
        .calculate(desiredChassisSpeeds.vxMetersPerSecond);
    desiredChassisSpeeds.vyMetersPerSecond = chassisSpeedsYSlewLimiter
        .calculate(desiredChassisSpeeds.vyMetersPerSecond);

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
        .toSwerveModuleStates(desiredChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        Constants.Swerve.maxAchievableVelocityMetersPerSecond);

    Logger.getInstance().recordOutput("drivetrain/desiredSwerveModuleStates", swerveModuleStates);

    if (closedLoop)
      setModuleStatesClosedLoop(swerveModuleStates);

    else
      setModuleStatesOpenLoop(swerveModuleStates);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxAchievableVelocityMetersPerSecond);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    }
  }

  public void setModuleStatesClosedLoopNoOptimize(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredStateNoOptimize(desiredStates[mod.moduleNumber], true);
    }
  }

  // useful for debugging
  public void setModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  // required by odometry
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  @Override
  public void periodic() {

    for (SwerveModule mod : mSwerveMods) {
      mod.periodic();
    }
    gyroIO.updateInputs(gyroInputs);
    if (gyroIO instanceof GyroIOSim) { // calculates sim gyro
      gyroIO.calculateYaw(getModulePositions());
    }

    updateOdometry();

    updatePoseEstimator();
    SmartDashboard.putNumber("pitch deg", getRobotPitchDegrees());
    SmartDashboard.putNumber("roll deg", getRobotRollDegrees());

    Logger.getInstance().recordOutput("drivetrain/odometryPose2d", getPoseMeters());
    Logger.getInstance().recordOutput("drivetrain/poseEstimatorPose2d", getEstimatorPoseMeters());
    Logger.getInstance().recordOutput(
        "drivetrain/swerveModuleStates",
        new SwerveModuleState[] {
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        });

  }

}