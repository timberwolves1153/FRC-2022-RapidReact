// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnForDegrees;
import frc.robot.commands.WaitCommand;
import frc.robot.lib.ShooterPosition;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallGatekeepHubLeftAutoGroup extends SequentialCommandGroup {
  /** Creates a new OneBallGatekeepHubLeftAutoGroup. */
  public OneBallGatekeepHubLeftAutoGroup(
    Trajectory oneBallTrajectory, 
    Supplier<Command> oneBallRamseteCommand, 
    Drive drive, 
    Launcher launcher, 
    Limelight limelight, 
    Collector collector
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.TARMAC), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new WaitCommand(1),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new TurnForDegrees(190, drive),
      new InstantCommand(() -> collector.setSolenoid(Value.kReverse), collector),
      new InstantCommand(() -> collector.collectIntake(), collector),
      oneBallRamseteCommand.get(),
      new InstantCommand(() -> collector.moverOff(), collector),
      new InstantCommand(() -> collector.singulatorOff(), collector),
      new InstantCommand(() -> collector.collectorStop(), collector),
      new InstantCommand(() -> collector.setSolenoid(Value.kForward), collector)
    );
  }
}
