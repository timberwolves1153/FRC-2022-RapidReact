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
public class FiveBallAutoGroup extends SequentialCommandGroup {
  /** Creates a new FiveBallAutoGroup. */
  public FiveBallAutoGroup(
  Trajectory fiveBallAutoTrajectory1,
  Trajectory fiveBallAutoTrajectory2,
  Trajectory fiveBallAutoTrajectory3,
  Supplier<RamseteCommand> fiveBallRamseteCommand1, 
  Supplier<RamseteCommand> fiveBallRamseteCommand2, 
  Supplier<RamseteCommand> fiveBallRamseteCommand3, 
  Collector collector, 
  Launcher launcher, 
  Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.TARMAC_LINE_HIGH), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory1.getInitialPose())),
      fiveBallRamseteCommand1.get(),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(180, drive),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(120, drive),
      new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory2.getInitialPose())),
      fiveBallRamseteCommand2.get(),
      new TurnForDegrees(180, drive),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory3.getInitialPose())),
      fiveBallRamseteCommand3.get(),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    );
  }
}
