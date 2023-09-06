// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;


/** Add your docs here. */
public class GyroIOReal implements GyroIO {

    private AHRS gyro;

    public GyroIOReal(Port kusb) {
        gyro = new AHRS(kusb);
        gyro.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        //these are correct believe it or not because of navX mounting issues
        inputs.robotPitchDegrees = gyro.getRoll();
        inputs.robotRollDegrees = gyro.getPitch();
        inputs.robotYawDegrees = gyro.getRotation2d().getDegrees();

        inputs.robotPitchDegreesPerSecond = gyro.getRawGyroY();
        inputs.robotRollDegreesPerSecond = gyro.getRawGyroX();
    };

    @Override
    public void zeroYaw() {
        gyro.zeroYaw();
    };
    

    /** Adds angle to yawDegrees. Does not clear upon usage of zeroYaw().
     * @param angle - the angle to add to yawDegrees.
     */
    @Override
    public void setAngleAdjustment(double angle) {
        gyro.setAngleAdjustment(angle);
    };
}