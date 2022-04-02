// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurnForDegrees;
import frc.robot.lib.ShooterPosition;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoRightGroup extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoRightGroup. */
  public TwoBallAutoRightGroup(Trajectory manualTrajectory1, Supplier<RamseteCommand> manualRamseteCommand1, Launcher launcher, Collector collector, Drive drive) {
    addCommands(
      new InstantCommand(() -> System.out.println("Running Partial Auto")),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(155, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      manualRamseteCommand1.get(),
      new TurnForDegrees(185, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      manualRamseteCommand1.get(),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.collectorStop(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector)
    );
  }
}
