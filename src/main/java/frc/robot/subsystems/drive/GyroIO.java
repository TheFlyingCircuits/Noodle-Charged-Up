// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public class GyroIOInputs {
        //TODO: add javadocs here to clarify continuity and robot vs. gyro-relativeness
        public double robotPitchDegrees = 0.0;
        public double robotRollDegrees = 0.0;

        //THIS ANGLE IS CONTINUOUS !!!!!!!!!!!!!!!!
        /** Continuous Angle */
        public double robotYawDegrees = 0.0;

        public double robotPitchDegreesPerSecond = 0.0;
        public double robotRollDegreesPerSecond = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {};

    public default void zeroYaw() {};
    
    /** Adds angle to yawDegrees. Does not clear upon usage of zeroYaw().
     * @param angle - the angle to add to yawDegrees.
     * 
     */
    public default void setAngleAdjustment(double angle) {};

    //this shouldn't need to be implemented by anything besides the sim implementation
    public default void calculateYaw(SwerveModulePosition[] modulePositions) {};
}