// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnForDegrees extends CommandBase {
  private Drive drive;
  private double degrees;
  private double startingDegrees;
  private double turnSpeed;

  private double sign;
  /** Creates a new TurnForDegrees. */
  public TurnForDegrees(double degrees, double turnSpeed, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.degrees = degrees;
    this.turnSpeed = turnSpeed;

    sign = Math.copySign(1.0, degrees);
    addRequirements(drive);
  }

  public TurnForDegrees(double degrees, Drive drive) {
    this(degrees, 0.5, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDegrees = drive.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(0, sign * turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(sign < 0) {
      return drive.getHeading() >= startingDegrees - degrees;
    } else {
      return drive.getHeading() <= startingDegrees - degrees;
    }
  }
}
