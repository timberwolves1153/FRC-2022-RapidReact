// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveForTime extends CommandBase {
  private double millisToDrive;
  private double direction;
  private Drive drive;

  private double initialTime;
  /** Creates a new DriveForTime. */
  public DriveForTime(double secondsToDrive, double direction, Drive drive) {
    this.millisToDrive = secondsToDrive * 1000;
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
    System.out.println("Running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(-direction * 0.7, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    System.out.println("Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - initialTime > millisToDrive;
  }
}
