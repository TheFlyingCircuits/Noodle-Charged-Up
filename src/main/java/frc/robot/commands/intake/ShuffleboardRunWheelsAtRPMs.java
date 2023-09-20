// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class ShuffleboardRunWheelsAtRPMs extends CommandBase {
  /** Creates a new ShuffleboardRunWheelsAtVolt. */
  private Intake intake;
  public ShuffleboardRunWheelsAtRPMs(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMotorRPMs(intake.getTopVolts(), intake.getBottomVolts());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotorRPMs(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
