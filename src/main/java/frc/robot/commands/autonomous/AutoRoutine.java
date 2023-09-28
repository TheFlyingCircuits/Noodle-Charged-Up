// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.intake.ShootCube;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine extends SequentialCommandGroup {
  /** Creates a new AutoRoutine. */
  public AutoRoutine(
      List<PathPlannerTrajectory> pathGroup, 
      Drivetrain drivetrain,
      Arm arm,
      Intake intake,
      Vision vision 
    ) {

    HashMap<String, Command> eventMap = new HashMap<>();

  // BALANCE ////////////////
    eventMap.put("waitOneSecond", new WaitCommand(1));
    
    eventMap.put("autoBalance", new AutoBalance(drivetrain));

  // SCORING
    eventMap.put("shootCubeHigh", new ShootCube(arm, intake, -6, -12));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getEstimatorPoseMeters,
      drivetrain::setPoseMeters,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(12.0, 0, 0),
      new PIDConstants(0.25, 0, 0),
      //new PIDConstants(0, 0, 0),
      //new PIDConstants(0, 0, 0),
      drivetrain::setModuleStatesClosedLoop,
      eventMap,
      true,
      drivetrain
    );
    
    addCommands(
      autoBuilder.fullAuto(pathGroup)
    );
  }
}