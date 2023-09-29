// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeWheelVoltages extends CommandBase {

  private Intake intake;
  double frontVolts;
  double backVolts;

  public SetIntakeWheelVoltages(Intake intake, double frontVolts, double backVolts) {
    addRequirements(intake);
    this.intake = intake;
    this.frontVolts=frontVolts;
    this.backVolts=backVolts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMotorVolts(frontVolts, backVolts);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setMotorVolts(0, 0);
  }
}
