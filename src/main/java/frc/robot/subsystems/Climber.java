// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private CANSparkMax winch;
  private CANSparkMax winch2;

  private DoubleSolenoid doubleSolenoid1;

  private DoubleSolenoid doubleSolenoid2;

  /** Creates a new Climber. */
  public Climber() {
    winch =  new CANSparkMax(4, MotorType.kBrushless);
    winch2 =  new CANSparkMax(5, MotorType.kBrushless);
    doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  }

  public void configSparkParams() {
    winch.restoreFactoryDefaults();
    winch2.restoreFactoryDefaults();

    winch.setIdleMode(IdleMode.kBrake);
    winch2.setIdleMode(IdleMode.kBrake);

    winch2.follow(winch, false);

    winch.burnFlash();
    winch2.burnFlash();
  }


  public void set(DoubleSolenoid.Value value) {
    doubleSolenoid1.set(value);
    doubleSolenoid2.set(value);
  }

  public void setRight(DoubleSolenoid.Value value) {
    doubleSolenoid1.set(value);
  }

  public void setLeft(DoubleSolenoid.Value value) {
    doubleSolenoid2.set(value);
  }

  public DoubleSolenoid.Value getSolenoid1State() {
    return doubleSolenoid1.get();
  }

  public DoubleSolenoid.Value getSolenoid2State() {
    return doubleSolenoid2.get();
  }

  public void toggle(){
    doubleSolenoid1.toggle();
    doubleSolenoid2.toggle();
  }

  //This is a helper method that clarifies what winching up means in the context of the set method; 
  //might need to be inverted depending on motor orientation
  public void winchUp() {
    set(0.8);
  }

  //Might need to be inverted depending on motor orientation
  public void winchDown() {
    set(-0.8);
  }

  public void winchUpRight() {
    setRight(0.8);
  }

  //Might need to be inverted depending on motor orientation
  public void winchDownRight() {
    setRight(-0.8);
  }
  public void winchUpLeft() {
    setLeft(0.8);
  }

  //Might need to be inverted depending on motor orientation
  public void winchDownLeft() {
    setLeft(-0.8);
  }

  //Helper method to stop the climber
  public void stop() {
    set(0);
  }

  //Main set method that can be called externally
  public void set(double speed) {
    winch.set(speed);
    winch2.set(speed);
  }
  public void setRight(double speed) {
    winch.set(speed);
  }
  public void setLeft(double speed) {
    winch2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
