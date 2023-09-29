// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class ReverseIntake extends CommandBase {
  /** Creates a new ReverseIntake. */
  private Intake intake;
  
  public ReverseIntake(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // arm.setArmPositionRadians(Constants.Arm.reverseIntakeAngleRadians);
    intake.setMotorVolts(-1.5, -1.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotorVolts(0, 0);
    // arm.setArmPositionRadians(Constants.Arm.maxAngleRadians);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
