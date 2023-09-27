// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCube extends SequentialCommandGroup {
  /** SequentialCommandGroup which moves the arm to shooting position and shoots. 
   * @param frontVolts - Voltage to send to the front set of wheels. Make this negative to shoot a cube.
   * @param backVolts - Voltage to send to the back set of wheels. Make this negative to shoot a cube.
  */
  public ShootCube(Arm arm, Intake intake, double frontVolts, double backVolts) {
    addCommands(
      new SetArmToPosition(arm, Constants.Arm.maxAngleRadians),
      new SetIntakeWheelVoltages(intake, frontVolts, backVolts).withTimeout(0.5),
      new SetIntakeWheelVoltages(intake, 0, 0)
      );
  }
}
