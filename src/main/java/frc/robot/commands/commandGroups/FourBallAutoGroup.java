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
public class FourBallAutoGroup extends SequentialCommandGroup {
  /** Creates a new FourBallAutoGroup. */
  public FourBallAutoGroup(
    Trajectory fourBallAutoTrajectory1, 
    Trajectory fourBallAutoTrajectory2,
    Trajectory fourBallAutoTrajectory4,
    Supplier<RamseteCommand> fourBallRamseteCommand1, 
    Supplier<RamseteCommand> fourBallRamseteCommand2,
    Supplier<RamseteCommand> fourBallRamseteCommand4,
    Collector collector, 
    Launcher launcher, 
    Drive drive
  ) {
    addCommands(
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.TARMAC_LINE_HIGH), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
      fourBallRamseteCommand1.get(),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(180, drive),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory2.getInitialPose())),
      fourBallRamseteCommand2.get(),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(180, drive),
      fourBallRamseteCommand4.get(),
      new InstantCommand(() -> collector.collectorStop(), collector),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() ->  collector.feederOn(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(() -> collector.moverOff(), collector),
    //   new TurnForDegrees(180, drive),
    //    new WaitCommand(0.25),
    //    new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory3.getInitialPose())),
    //    generateRamseteCommandFromTrajectory(fourBallAutoTrajectory3),
    //    new TurnForDegrees(160, drive),
    //   new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory4.getInitialPose())),
    //    generateRamseteCommandFromTrajectory(fourBallAutoTrajectory4),
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
    //   new InstantCommand(() -> launcher.pidOn(), launcher),
    //   new InstantCommand(() -> collector.moverForward(), collector),
    //   new InstantCommand(() -> collector.feederOn(), collector),
    //   new WaitCommand(1),
    //   new InstantCommand(() -> launcher.pidOff(), launcher),
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(()-> collector.moverOff(), collector),
    //   new InstantCommand(()-> collector.singulatorStop(), collector),
    //   new InstantCommand(()-> collector.collectorStop(), collector)
    );
  }

  public FourBallAutoGroup(Trajectory fourBallAutoTrajectory1, Trajectory fourBallAutoTrajectory2,
      Trajectory fourBallAutoTrajectory4, Supplier<RamseteCommand> supplier, Supplier<RamseteCommand> supplier2,
      Object object, Collector collector, Launcher launcher, Drive drive) {
  }
}
