// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.SetIntakeWheelSpeeds;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOReal;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;

public class RobotContainer {

  public List<PathPlannerTrajectory> pathGroup;

  // CONTROLLERS
  public static final CommandXboxController controller = new CommandXboxController(0);

  // SUBSYSTEMS
  public final Drivetrain drivetrain;
  public final Vision vision;
  public final Arm arm;
  public final Intake intake;
  public final SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    if (RobotBase.isReal()) {
      drivetrain = new Drivetrain(
          new GyroIOReal(Port.kMXP),
          new SwerveModuleIOReal(
              Constants.Swerve.FrontLeftSwerveModule.driveMotorID,
              Constants.Swerve.FrontLeftSwerveModule.steerMotorID,
              Constants.Swerve.FrontLeftSwerveModule.steerEncoderID,
              Constants.Swerve.FrontLeftSwerveModule.steerOffset),
          new SwerveModuleIOReal(
              Constants.Swerve.FrontRightSwerveModule.driveMotorID,
              Constants.Swerve.FrontRightSwerveModule.steerMotorID,
              Constants.Swerve.FrontRightSwerveModule.steerEncoderID,
              Constants.Swerve.FrontRightSwerveModule.steerOffset),
          new SwerveModuleIOReal(
              Constants.Swerve.BackLeftSwerveModule.driveMotorID,
              Constants.Swerve.BackLeftSwerveModule.steerMotorID,
              Constants.Swerve.BackLeftSwerveModule.steerEncoderID,
              Constants.Swerve.BackLeftSwerveModule.steerOffset),
          new SwerveModuleIOReal(
              Constants.Swerve.BackRightSwerveModule.driveMotorID,
              Constants.Swerve.BackRightSwerveModule.steerMotorID,
              Constants.Swerve.BackRightSwerveModule.steerEncoderID,
              Constants.Swerve.BackRightSwerveModule.steerOffset));

      vision = new Vision(new VisionIOReal());

      arm = new Arm(new ArmIOReal());

      intake = new Intake(new IntakeIOReal());

    } else if (RobotBase.isSimulation()) {
      drivetrain = new Drivetrain(
          new GyroIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());

      vision = new Vision(new VisionIO() {
      });

      arm = new Arm(new ArmIOSim());

      intake = new Intake(new IntakeIO() {});

    } else {
      drivetrain = new Drivetrain(
          new GyroIO() {
          },
          new SwerveModuleIO() {
          },
          new SwerveModuleIO() {
          },
          new SwerveModuleIO() {
          },
          new SwerveModuleIO() {
          });

      vision = new Vision(new VisionIO() {
      });

      arm = new Arm(new ArmIO() {});

      intake = new Intake(new IntakeIO() {});
    }

    configureBindings();



    autoChooser.setDefaultOption("1 cone + balance wire guard", "1 cone + balance wire guard");



    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, true));

    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {
    controller.y().onTrue(new InstantCommand(drivetrain::zeroYaw));
    controller.button(1).onTrue(new InstantCommand(arm::setArmPosition45Degrees));
    controller.button(2).onTrue(new InstantCommand(arm::setArmPosition0Degrees));
    controller.rightTrigger().onTrue(new SetIntakeWheelSpeeds(intake, 100, 100));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(drivetrain::setPose2D180);//.andThen(
        // new AutoRoutine(pathGroup, drivetrain, vision));
  }
}
