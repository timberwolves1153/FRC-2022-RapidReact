// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorSensor;

public class SmartBallShoot extends CommandBase {

  private Collector collector;
  private ColorSensor colorSensor;

  /** Creates a new SmartBallShoot. */
  public SmartBallShoot(Collector collector, ColorSensor colorSensor) {
    this.collector = collector;
    this.colorSensor = colorSensor;

    addRequirements(collector, colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.feederOn();
    collector.singulatorIntake();
    //if(colorSensor.getDetectedBallColor().equals(Robot.getContainer().getSelectedAllianceColor()))
    if(collector.moverSeesBall() && !collector.feederSeesBall()){
      collector.moverForward();
    }else{
      collector.moverOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.feederOff();
    collector.moverOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
