// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static enum LoggerMode {
        ROBOT_SIM,
        ROBOT_REAL_LOGGED,
        ROBOT_REAL,
        ROBOT_REPLAY
    }

    public static final class Swerve {
        // KINEMATICS CONSTANTS
        /**
         * Distance between the center point of the left wheels and the center point of
         * the right wheels.
         */
        public static final double trackwidthMeters = Units.inchesToMeters(22.75);
        /**
         * Distance between the center point of the front wheels and the center point of
         * the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(22.75);

        public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelDiamaterMeters = 0.10033;
        public static final double wheelCircumferenceMeters = wheelDiamaterMeters * Math.PI;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
                new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
                new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
                new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));

        // PID + FEEDFORWARD CONSTANTS FOR MOTORS

        // PID for drive motors.
        public static final double drivekPVoltsPerMeterPerSecond = 0.81027; // sysid says this should 0.27
        public static final double drivekIVoltsPerMeter = 0.;
        public static final double drivekDVoltsPerMeterPerSecondSquared = 0.00;

        // PID for angle motors.
        public static final double anglekPVoltsPerDegree = 0.08;// 0.065;
        public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk
        public static final double anglekDVoltsPerDegreePerSecond = 0.;

        public static final double drivekSVolts = 0.1301;
        public static final double drivekVVoltsSecondsPerMeter = 2.6931; // .8679
        public static final double drivekAVoltsSecondsSquaredPerMeter = 0.43963;

        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight
         * line, based off of the empirical free speed of the drive NEOs.
         */
        public static final double maxAchievableVelocityMetersPerSecond = 5880.0 / 60.0 *
                driveReduction *
                wheelDiamaterMeters * Math.PI;

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this
         * speed.
         * <br>
         * This value will be less than or equal to the
         * maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = 4.3;

        /**
         * The maximum achievable angular velocity of the robot in radians per second.
         * <br>
         * This is a measure of how fast the robot can rotate in place, based off of
         * maxAchievableVelocityMetersPerSecond.
         */
        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond /
                Math.hypot(Constants.Swerve.trackwidthMeters / 2.0, Constants.Swerve.wheelbaseMeters / 2.0);

        /**
         * This is the max desired angular velocity that will be achievable in teleop.
         * <br>
         * If the controller rotation joystick is maxed in one direction, it will rotate
         * at this speed.
         * <br>
         * This value will be tuned based off of driver preference.
         */
        public static final double maxDesiredAngularVelocityRadiansPerSecond = 5.4;

        public static final double maxDesiredDriverAccel = 27.27;

        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /**
         * Offset for the angle CANCoder encoder.
         * <br>
         * This offsets the angle encoder so that it reads as 0 when pointing forwards.
         * <br>
         * All values are relative to the front left corner configuration.
         */
        public static final class CANCoderOffsets {
            public static final double one = 210.43;//0;//85.693;
            public static final double two = 108.896;
            public static final double three = 17.23 - 90;
            public static final double four = 0;
            public static final double five = 105.82;
            public static final double six = 85.43;
            public static final double seven = 99.492;
            public static final double eight = 0;
            public static final double nine = 0;
        }

        // This enum is used to determine the offset for the swerve modules.
        public enum SwerveModuleCorners {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }

        /**
         * This method is used to get the offset for the swerve modules.
         * 
         * @param leftRightOffset the offset for the left and right swerve modules.
         *                        This should be a {@link CANCoderOffsets} constant.
         * @param corner          the corner of the swerve module to get the offset for.
         * @return the angular offset for the swerve module in degrees.
         */
        public static double getOffset(double leftRightOffset, SwerveModuleCorners corner) {
            switch (corner) {
                case FRONT_LEFT:
                    return leftRightOffset;
                case FRONT_RIGHT:
                    return leftRightOffset - 90;
                case BACK_LEFT:
                    return leftRightOffset + 90;
                case BACK_RIGHT:
                    return leftRightOffset + 180;
                default:
                    return leftRightOffset;
            }
        }

        public static final class FrontLeftSwerveModule {
            /** CAN ID for the SPARK MAX used to control the front left driving NEO */
            public static final int driveMotorID = 1;
            /** CAN ID for the SPARK MAX used to control the front left steering NEO */
            public static final int steerMotorID = 2;
            /** CAN ID for the front left CANcoder */
            public static final int steerEncoderID = 3;
            /** Offset from true zero for the front left swerve module in degrees */
            public static final double steerOffset = Constants.Swerve.getOffset(
                    CANCoderOffsets.one, SwerveModuleCorners.FRONT_LEFT);
        }

        public static final class FrontRightSwerveModule {
            /** CAN ID for the SPARK MAX used to control the front right driving NEO */
            public static final int driveMotorID = 3;
            /** CAN ID for the SPARK MAX used to control the front right steering NEO */
            public static final int steerMotorID = 4;
            /** CAN ID for the front right CANcoder */
            public static final int steerEncoderID = 4;
            /** Offset from true zero for the front right swerve module in degrees */
            public static final double steerOffset = Constants.Swerve.getOffset(
                    CANCoderOffsets.four,
                    SwerveModuleCorners.FRONT_RIGHT);

        }

        public static final class BackLeftSwerveModule {
            /** CAN ID for the SPARK MAX used to control the back left driving NEO */
            public static final int driveMotorID = 5;
            /** CAN ID for the SPARK MAX used to control the back left steering NEO */
            public static final int steerMotorID = 6;
            /** CAN ID for the back left CANcoder */
            public static final int steerEncoderID = 6;
            /** Offset from true zero for the back left swerve module in degrees */
            public static final double steerOffset = Constants.Swerve.getOffset(
                    CANCoderOffsets.six,
                    SwerveModuleCorners.BACK_LEFT);
        }

        public static final class BackRightSwerveModule {
            /** CAN ID for the SPARK MAX used to control the back right driving NEO */
            public static final int driveMotorID = 7;
            /** CAN ID for the SPARK MAX used to control the back right steering NEO */
            public static final int steerMotorID = 8;
            /** CAN ID for the back right CANcoder */
            public static final int steerEncoderID = 5;
            /** Offset from true zero for the back right swerve module in degrees */
            public static final double steerOffset = Constants.Swerve.getOffset(
                    CANCoderOffsets.five,
                    SwerveModuleCorners.BACK_RIGHT);
        }
    }

    public final static class Arm {
        public static final double minAngleRadians = Units.degreesToRadians(-5);
        public static final double maxAngleRadians = Units.degreesToRadians(144);

        public static final double armWidthMeters = Units.inchesToMeters(50);
        public static final double armLengthMeters = Units.inchesToMeters(50);
        
        public static final double maxDesiredAccelerationRadiansPerSecond = 6.0;//1;//3;
        public static final double maxDesiredVelocityRadiansPerSecond = 7.0;//1;//7;

        public static final Constraints motionConstraints = new Constraints(
            Constants.Arm.maxDesiredVelocityRadiansPerSecond,
            Constants.Arm.maxDesiredAccelerationRadiansPerSecond);

        public static final double gearReduction = 105;
        // motors
        public static final int leftPivotMotorID = 9;
        public static final boolean leftPivotMotorInverted = false;
        public static final int rightPivotMotorID = 15;
        public static final boolean rightPivotMotorInverted = true;
        public static final double pivotMotorContinuousCurrentLimit = 20;
        public static final IdleMode pivotIdleMode = IdleMode.kBrake;

        //PID

        public static final double KpVoltsPerRadianPerSecond = 0.510;
        public static final double KiVoltsPerRadian = 0.7; 
        public static final double KdVoltsPerRadianPerSecondSquared = 0.00;

        public static final double KsVolts = 0.062;
        public static final double KgVolts = 0.455;//0.33 FOR SIM;
        public static final double KvVoltsPerRadianPerSecond = 1.073;//1 FOR SIM;
        public static final double KaVoltsPerRadianPerSecondSquared = 0.024;
    }

    public static class Controller {

        public static final double controllerDeadzone = 0.175;

        public static int buttonA = 1;
        public static int buttonB = 2;
        public static int buttonX = 3;
        public static int buttonY = 4;
        public static int leftBumper = 5;
        public static int rightBumper = 6;
        public static int viewButton = 7;
        public static int menuButton = 8;
        public static int leftJoyStickPress = 9;
        public static int rightJoyStickPress = 10;
    }

    public static class Vision {
        public static enum LimelightTarget {
            aprilTag(0, 0.46, 0.36),
            midTape(1, 0.61, 0.59),
            highTape(2, 1.11, 1.01);

            public int limelightPipeline;
            public double heightMeters;
            public double distanceMeters;

            LimelightTarget(int limelightPipeline, double heightMeters, double distanceMeters) {
                this.limelightPipeline = limelightPipeline;
                this.heightMeters = heightMeters;
                this.distanceMeters = distanceMeters;
            }
        }

        /** Height of the Limelight from the ground. */
        public static final double limelightHeightMeters = Units.inchesToMeters(22.75);
        /** Angle above the horizontal of the Limelight. */
        public static final double limelightAngleDegrees = -9.37;
        /** Lateral offset of the Limelight from the center of the robot. */
        public static final double limelightLateralOffsetMeters = -0.27;
        /** Horizontal distance from limelight to front bumper. */
        public static final double limelightBumperDistanceMeters = 0.315;
    }
}
