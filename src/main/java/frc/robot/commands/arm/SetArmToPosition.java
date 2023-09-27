// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class SetArmToPosition extends CommandBase {
  private Arm arm;
  private double setpoint;
  public SetArmToPosition(Arm arm, double setpointRadians) {
    this.arm = arm;
    this.setpoint = setpointRadians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmPositionRadians(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atSetpoint();
  }
}
