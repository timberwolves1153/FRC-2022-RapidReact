// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterA;
  private CANSparkMax shooterB;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterA = new CANSparkMax(10, MotorType.kBrushless);
    shooterB = new CANSparkMax(11, MotorType.kBrushless);

    configSparkParams();
  }

  public void configSparkParams() {
    shooterA.restoreFactoryDefaults();
    shooterB.restoreFactoryDefaults();

    shooterA.setIdleMode(IdleMode.kCoast);
    shooterB.setIdleMode(IdleMode.kCoast);

    shooterB.follow(shooterA, true);

    shooterA.burnFlash();
    shooterB.burnFlash();
  }

  public void setSpeed(double speed) {
    shooterA.set(speed);
  }

  public void stop() {
    shooterA.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
