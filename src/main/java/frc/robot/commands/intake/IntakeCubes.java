// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class IntakeCubes extends CommandBase {
  /** Creates a new IntakeCubes. */
  private Intake intake;
  private Arm arm;
  public IntakeCubes(Intake intake, Arm arm) {
    addRequirements(intake);
    addRequirements(arm);
    this.intake = intake;
    this.arm = arm;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMotorVolts(4, 4);
    arm.setArmPositionRadians(Units.degreesToRadians(-5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotorVolts(0, 0);
    // arm.setArmPositionRadians(Constants.Arm.targetMaxAngleRadians);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
