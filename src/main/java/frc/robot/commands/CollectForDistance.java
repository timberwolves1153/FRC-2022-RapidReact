// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectForDistance extends CommandBase {
  private Collector collect;
  private double distance;
  /** Creates a new ClimbForDistance. */
  public CollectForDistance(double distance, Collector collect) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collect = collect;
    this.distance = distance;

    addRequirements(collect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collect.resetRightCollectorEncoders();
    collect.setWinchPosition(collect.getEncoderTicksFromPosition(distance));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collect.collectorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
