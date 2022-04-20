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
import frc.robot.lib.ShooterPosition;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoLeftGroup extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoLeftGroup. */
  public TwoBallAutoLeftGroup(
    Trajectory fourBallAutoTrajectory1, 
    Supplier<Command> ramseteCommand1, 
    Launcher launcher, 
    Collector collector, 
    Drive drive
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> System.out.println("Running Three Ball Auto")),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(()-> collector.feederOn(), collector),
      new WaitCommand(1),
      new InstantCommand(()-> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new TurnForDegrees(170, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(() -> collector.collectIntake(), collector),
      //new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
      ramseteCommand1.get(),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorOff(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    );
  }
}
