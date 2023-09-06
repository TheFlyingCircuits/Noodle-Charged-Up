// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;



public class AutoBalance extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final PIDController tiltController;

  public AutoBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    tiltController = new PIDController(0.5/10, 0, 0.0045); 
    tiltController.setSetpoint(0);
    tiltController.setTolerance(1.0, 15);

    addRequirements(drivetrain);


    
    
    SmartDashboard.putBoolean("balance 2nd stage activated", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tiltController.setP(0.5/10);
    SmartDashboard.putBoolean("balance 2nd stage activated", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(drivetrain.getRobotPitchDegreesPerSecond()) > 35 || Math.abs(drivetrain.getRobotPitchDegrees()) < 5) {
      tiltController.setP(0.17/10);
      SmartDashboard.putBoolean("balance 2nd stage activated", true);
    }
    
    double desiredVxMetersPerSecond = -tiltController.calculate(drivetrain.getRobotPitchDegrees());
    drivetrain.drive(
      new ChassisSpeeds(
        desiredVxMetersPerSecond,
        0,
        0),
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0), true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}