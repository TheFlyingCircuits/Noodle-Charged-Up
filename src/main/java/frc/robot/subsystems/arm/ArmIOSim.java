// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

    private double appliedVolts = 0;
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getNeo550(2),
        Constants.Arm.gearReduction,
        // Converts from lb-in^2 to kg-m^2 probably correctly
        (635.543*0.000292639686), //0.185985103959
        Units.inchesToMeters(19.25),
        Constants.Arm.minAngleRadians,
        Constants.Arm.maxAngleRadians,
        true
        );

        public ArmIOSim() {
            System.out.println("[Init] Creating " + this.getClass().getName());
            // revPhysicsSim = REVPhysicsSim.getInstance();
            
            // revPhysicsSim.addSparkMax(null, DCMotor.getNeo550(1));

        }

    public void setArmVoltage(double volts) {
        armSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
        appliedVolts = (MathUtil.clamp(volts, -12, 12));
    }

    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(0.02);
        inputs.armPosition = new Rotation2d(armSim.getAngleRads());
        inputs.armVelocityRadiansPerSecond = armSim.getVelocityRadPerSec();
        inputs.leftPivotMotorArmVolts = appliedVolts;
        inputs.rightPivotMotorArmVolts = appliedVolts;
        inputs.leftPivotMotorArmCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.rightPivotMotorArmCurrentAmps = armSim.getCurrentDrawAmps();
        // Just room temp lol
        inputs.leftPivotMotorTempCelsius = 20;
        inputs.rightPivotMotorTempCelsius = 20;

        inputs.atFrontLimitSwitch = armSim.hasHitLowerLimit();
        inputs.atBackLimitSwitch = armSim.hasHitUpperLimit();
    }
}
