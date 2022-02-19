// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PartialAutoCommandGroup extends SequentialCommandGroup {
  /** Creates a new PartialAutoCommandGroup. */
  public PartialAutoCommandGroup(
  RamseteCommand manualRamseteCommand1, 
  Trajectory manualTrajectory1, 
  Launcher launcher, 
  Collector collector, 
  Drive drive) {
    addCommands(
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> launcher.feederOn(), launcher),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> launcher.feederOff(), launcher),
      new TurnForDegrees(165, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      manualRamseteCommand1
    );
  }
}
