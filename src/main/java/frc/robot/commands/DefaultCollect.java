// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class DefaultCollect extends CommandBase {
  private DoubleSupplier leftTrigger;
  private DoubleSupplier rightTrigger;
  private Collector collector;

  /** Creates a new DefaultCollect. */
  public DefaultCollect(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, Collector collector) {
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(leftTrigger.getAsDouble() > 0.1) {
       collector.singulatorIntake();
       collector.moverForward();
      //collector.smartBallCollect();
    }
    if(rightTrigger.getAsDouble() > 0.1) {
      collector.singulatorOutake();
      collector.moverReverse();
    }
    if(leftTrigger.getAsDouble() < 0.1 && rightTrigger.getAsDouble() < 0.1) {
      collector.singulatorStop();
      collector.moverOff();
      collector.feederOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
