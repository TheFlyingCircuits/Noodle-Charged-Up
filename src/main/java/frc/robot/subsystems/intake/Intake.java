// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  
  private PIDController frontPIDController;
  private PIDController backPIDController;

  private SimpleMotorFeedforward feedforward;

  ShuffleboardTab tab;
  GenericEntry topVolts;
  GenericEntry bottomVolts;
  public Intake(IntakeIO io) {
    this.io=io;
    this.inputs = new IntakeIOInputsAutoLogged();

    tab = Shuffleboard.getTab("rpmvolts");
    topVolts = Shuffleboard.getTab("rpmvolts")
      .add("top volts", 0)
      .getEntry();

    bottomVolts = Shuffleboard.getTab("rpmvolts")
      .add("bottom volts", 0)
      .getEntry();

    frontPIDController = new PIDController(0, 0, 0);
    backPIDController = new PIDController(0, 0, 0);

    feedforward = new SimpleMotorFeedforward(0.001, .003, 0.5); //units of VOLT per RPM
  }

  public void setMotorRPMs(double frontRPM, double backRPM) {
    double frontFeedforwardOutputVolts = feedforward.calculate(frontRPM);
    double backFeedforwardOutputVolts = feedforward.calculate(backRPM);
    double frontPIDOutputVolts = frontPIDController.calculate(inputs.frontVelocityRPM);
    double backPIDOutputVolts = backPIDController.calculate(inputs.backVelocityRPM);

    io.setIntakeWheelVoltages(frontFeedforwardOutputVolts + frontPIDOutputVolts, backFeedforwardOutputVolts + backPIDOutputVolts);
  }

  public void setMotor100RPM() {
    setMotorRPMs(100, 100);
  }


  public double getTopVolts() {
    return topVolts.getDouble(0);
  }
  public double getBottomVolts() {
    return bottomVolts.getDouble(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.getInstance().processInputs("Intake", inputs);

  }
}
