// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurnForDegrees;
import frc.robot.commands.TurnWithLimelightV2;
import frc.robot.lib.ShooterPosition;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAutoGroup extends SequentialCommandGroup {
  /** Creates a new FourBallAutoGroup. */
  public FourBallAutoGroup(
    Trajectory fourBallAutoTrajectory1, 
    Trajectory fourBallAutoTrajectory2,
    Trajectory fourBallAutoTrajectory3,
    Supplier<Command> fourBallRamseteCommand1, 
    Supplier<Command> fourBallRamseteCommand2,
    Supplier<Command> fourBallRamseteCommand3,
    Collector collector, 
    Launcher launcher, 
    Drive drive,
    Limelight limelight
  ) {
    addCommands(
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.LINE), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      fourBallRamseteCommand1.get(),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(177, drive),
      new TurnWithLimelightV2(1.5, drive, limelight),
      // new ParallelCommandGroup(
      //   new SequentialCommandGroup(
      //     new WaitCommand(0.25),
      //     new InstantCommand(() -> collector.feederOn(), collector),
      //     new InstantCommand(() -> collector.moverForward(), collector)
      //   ),
      //   new DriveDistance(0.5, drive)
      // ),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new WaitCommand(0.75),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(-168, drive),
      // new InstantCommand(() -> collector.feederOff(), collector),
      //new InstantCommand(() -> collector.moverForward(), collector),
      // fourBallRamseteCommand2.get(),
      // new WaitCommand(1),
      // new InstantCommand(()-> collector.moverOff(), collector),
      // fourBallRamseteCommand3.get(),
      // new TurnForDegrees(177, drive),
      // new TurnWithLimelightV2(1, drive, limelight),
      // new InstantCommand(() -> collector.feederOn(), collector),
      // new InstantCommand(() -> collector.moverForward(), collector),
      // new WaitCommand(0.75),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorOff(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    );
  }
}
