// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.ShooterPosition;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;

public class Shoot extends CommandBase {
  private Launcher launcher;
  private Limelight limelight;
  private boolean isAuto;
  private double initialTime;
  private double runtimeMillis;

  /** Creates a new Shoot. */
  public Shoot(boolean isAuto, double runtimeSeconds, Launcher launcher, Limelight limelight) {
    this.launcher = launcher;
    this.limelight = limelight;
    this.isAuto = isAuto;
    this.runtimeMillis = runtimeSeconds * 1000;

    addRequirements(launcher, limelight);
  }

  public Shoot(double runtimeSeconds, Launcher launcher, Limelight limelight) {
    this(true, runtimeSeconds, launcher, limelight);
  }

  public Shoot(Launcher launcher, Limelight limelight) {
    this(false, 0, launcher, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.pidOn();

    if(!launcher.getOverride()) {
      if(limelight.calcDistance() < 53) {
        launcher.setGainPreset(ShooterPosition.FENDER_HIGH);
      } else if(limelight.calcDistance() > 53 && limelight.calcDistance() < 83) {
        launcher.setGainPreset(ShooterPosition.TARMAC);
      } else if(limelight.calcDistance() > 83 && limelight.calcDistance() < 120) {
        launcher.setGainPreset(ShooterPosition.LINE);
      } else if(limelight.calcDistance() > 120) {
        launcher.setGainPreset(ShooterPosition.LAUNCHPAD);
      }
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.pidOff();
    System.out.println("Ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isAuto){
      return System.currentTimeMillis() - initialTime > runtimeMillis;
    } 
    return false;
  }
}
