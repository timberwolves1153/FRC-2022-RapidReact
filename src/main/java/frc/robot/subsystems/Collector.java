// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Collector extends SubsystemBase {

  private WPI_TalonFX collect;
  private DoubleSolenoid doubleSolenoid1;
  private CANSparkMax mover;
  private CANSparkMax singulator;
  private DigitalInput moverSensor;
  private DigitalInput singulatorSensor;
  

  /** Creates a new Collector.*/
  public Collector() {
    collect = new WPI_TalonFX(10);
    singulator = new CANSparkMax(11, MotorType.kBrushless);
    mover = new CANSparkMax(12, MotorType.kBrushless);

    doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    moverSensor = new DigitalInput(0);
    singulatorSensor = new DigitalInput(1);
  }

  

  public void moverForward() {
    mover.set(0.5);
  }

  public void moverOff() {
    mover.set(0);
  }

  public void moverReverse() {
    mover.set(-0.5);
  }
  
  //This is a helper method that clarifies what winching up means in the context of the set method; 
  //might need to be inverted depending on motor orientation
  public void intake() {
    collect.set(1);
  }

  //Might need to be inverted depending on motor orientation
  public void outake() {
    collect.set(-1);
  }

  public void singulatorIntake() {
    singulator.set(1);
  }

  public void singulatorOutake() {
    singulator.set(-1);
  }

  public void singulatorStop() {
    singulator.set(0);
  }

  //Helper method to stop the climber
  public void stop() {
    set(0);
  }

  //Main set method that can be called externally
  public void set(double speed) {
    collect.set(speed);
  }

  public void setSolenoid(DoubleSolenoid.Value value) {
    doubleSolenoid1.set(value);
  }

  public DoubleSolenoid.Value getSolenoid1State() {
    return doubleSolenoid1.get();
  }

  public void toggleSolenoid(){
    doubleSolenoid1.toggle();
    if(doubleSolenoid1.get().equals(DoubleSolenoid.Value.kOff)) {
      setSolenoid(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getMoverSensor(){
    return moverSensor.get();
  }

  public boolean getSingulatorSensor(){
    return singulatorSensor.get();
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
