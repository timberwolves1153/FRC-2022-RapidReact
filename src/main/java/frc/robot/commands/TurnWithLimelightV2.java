// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class TurnWithLimelightV2 extends CommandBase {
  private Limelight limelight;
  private Drive drive;

  private double degrees;
  private double sign;
  private double startingDegrees;

  private double maxRunTimeSeconds;
  private double startTime;

  /** Creates a new TurnWithLimelightV2. */
  public TurnWithLimelightV2(double maxRunTimeSeconds, Drive drive, Limelight limelight) {
    this.drive = drive;
    this.limelight = limelight;
    this.maxRunTimeSeconds = maxRunTimeSeconds;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    degrees = limelight.getTargetValues().x;
    sign = Math.copySign(1, degrees);

    startingDegrees = drive.getHeading();

    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    System.out.println(limelight.targetExists());

    if(limelight.targetExists()) {
      if(!finishedTurning()) {
        if(Math.abs(limelight.getTargetValues().x) > 10) {
          speed = 0.5;
        } else if(Math.abs(limelight.getTargetValues().x) > 5) {
          speed = 0.4;
        } else {
          speed = 0.32;
        }
    
        drive.arcadeDrive(0, sign * speed);
  
      } else {
        drive.arcadeDrive(0, 0);
        degrees = limelight.getTargetValues().x;
        sign = Math.copySign(1, degrees);
  
        startingDegrees = drive.getHeading();
      }
    } else {
      drive.arcadeDrive(0, 0.5);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() - startTime > maxRunTimeSeconds * 1000) {
      return true;
    }
    if(limelight.getTargetValues().x > -3 && limelight.getTargetValues().x < 3 && finishedTurning()) 
      return true;

    return false;
  }

  private boolean finishedTurning() {
    if(sign < 0) {
      return drive.getHeading() >= startingDegrees - degrees;
    } else {
      return drive.getHeading() <= startingDegrees - degrees;
    }
  }
}
