// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class ShuffleboardRunWheelsAtVolt extends CommandBase {
  /** Creates a new ShuffleboardRunWheelsAtVolt. */
  private Intake intake;
  public ShuffleboardRunWheelsAtVolt(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (intake.getTopVolts() > 0) intake.setMotorAmpLimits(15, 15);
    else intake.setMotorAmpLimits(40, 40);
    intake.setMotorVolts(intake.getTopVolts(), intake.getBottomVolts());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotorVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
