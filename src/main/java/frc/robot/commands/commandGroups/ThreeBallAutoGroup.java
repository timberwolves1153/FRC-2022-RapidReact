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
public class ThreeBallAutoGroup extends SequentialCommandGroup {
  /** Creates a new ThreeBallAutoGroup. */
  public ThreeBallAutoGroup(
    Trajectory threeBallAutoTrajectory1, 
    Trajectory threeBallAutoTrajectory2, 
    Trajectory threeBallAutoTrajectory3, 
    Supplier<Command> threeBallRamseteCommand1, 
    Supplier<Command> threeBallRamseteCommand2, 
    Supplier<Command> threeBallRamseteCommand3,
    Collector collector, 
    Drive drive, 
    Launcher launcher,
    Limelight limelight
  ) {
    
    addCommands(
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(()-> collector.feederOn(), collector),
      new WaitCommand(1),
      new InstantCommand(()-> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new TurnForDegrees(179, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(() -> collector.collectIntake(), collector),
      //new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
      threeBallRamseteCommand1.get(),
      new TurnForDegrees(97, drive),
      //new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory2.getInitialPose())),
      threeBallRamseteCommand2.get(),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
    //  new InstantCommand(()-> collector.collectorStop(), collector),
     // new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kForward)),
      new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.LINE), launcher),
      new TurnForDegrees(105, drive), 
      //new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory3.getInitialPose())),
      threeBallRamseteCommand3.get(),
      new TurnWithLimelightV2(2, drive, limelight),
     // new TurnWithLimeLight(drive, limelight),
      //generateRamseteCommandFromTrajectory(threeBallAutoTrajectory3),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(()-> collector.moverForward(), collector),
      new InstantCommand(()-> collector.feederOn(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    );
  }
}
