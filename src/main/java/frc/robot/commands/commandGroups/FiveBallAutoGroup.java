// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  //Trajectory fiveBallAutoTrajectory4,
  Supplier<Command> fiveBallRamseteCommand1, 
  Supplier<Command> fiveBallRamseteCommand2, 
  Supplier<Command> fiveBallRamseteCommand3, 
  //Supplier<Command> fiveBallRamseteCommand4, 
  Collector collector, 
  Launcher launcher, 
  Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new InstantCommand(() -> {
      //   drive.setFieldTrajectory("fiveBallAuto1", fiveBallAutoTrajectory1);
      //   drive.setFieldTrajectory("fiveBallAuto2", fiveBallAutoTrajectory2);
      //   drive.setFieldTrajectory("fiveBallAuto3", fiveBallAutoTrajectory3);
      // }, drive),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.LINE), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      //new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory1.getInitialPose())),
      fiveBallRamseteCommand1.get(),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(177, drive),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new WaitCommand(0.5),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(-95, drive),
     // new InstantCommand(() -> drive.resetOdometry(fiveBallAutoTrajectory2.getInitialPose())),
      fiveBallRamseteCommand2.get(),
      new WaitCommand(0.5),
      new TurnForDegrees(-155, drive),
      new InstantCommand(()-> collector.moverOff(), collector),
      //new InstantCommand(()-> drive.resetOdometry(fiveBallAutoTrajectory3.getInitialPose())),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(2),
          new InstantCommand(() -> collector.feederOn(), collector),
          new InstantCommand(() -> collector.moverForward(), collector)
          ),
        fiveBallRamseteCommand3.get()
      ),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorOff(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    );
  }
}
