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
public class GatekeepAutoGroup extends SequentialCommandGroup {
  /** Creates a new GateKeepAutoGroup. */
  public GatekeepAutoGroup(
    Trajectory gatekeepPathTrajectory1, 
    Trajectory gatekeepPathTrajectory2, 
    Trajectory gatekeepPathTrajectory3, 
    Supplier<Command> gatekeepRamseteCommand1, 
    Supplier<Command> gatekeepRamseteCommand2, 
    Supplier<Command> gatekeepRamseteCommand3,
    Collector collector, 
    Drive drive, 
    Launcher launcher
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(185, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      //new InstantCommand(()-> drive.resetOdometry(gatekeepPathTrajectory1.getInitialPose())),
      gatekeepRamseteCommand1.get(),
      new TurnForDegrees(160, drive),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.LAUNCHPAD), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(1.5),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(-85, drive),
      //new InstantCommand(()-> drive.resetOdometry(gatekeepPathTrajectory2.getInitialPose())),
      gatekeepRamseteCommand2.get(),
      new TurnForDegrees(-120, drive),
      new InstantCommand(() -> collector.collectOutake(), collector),
      new InstantCommand(() -> collector.singulatorOutake(), collector),
      new InstantCommand(() -> collector.moverReverse(), collector),
      new WaitCommand(0.25),
      new TurnForDegrees(-50, drive),
      //new InstantCommand(() -> drive.resetOdometry(gatekeepPathTrajectory3.getInitialPose())),
      gatekeepRamseteCommand3.get(),
      new WaitCommand(0.5),
      new InstantCommand(() -> collector.collectOutake(), collector),
      new InstantCommand(() -> collector.singulatorOutake(), collector),
      new InstantCommand(() -> collector.moverReverse(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH), launcher),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> collector.moverOff(), collector),
      new InstantCommand(() -> collector.singulatorOff(), collector)
    );
  }
}
