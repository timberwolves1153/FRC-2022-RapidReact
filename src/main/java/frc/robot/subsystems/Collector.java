// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Collector extends SubsystemBase {

  private WPI_TalonFX collect;
  private DoubleSolenoid doubleSolenoid1;
  private WPI_TalonFX indexerA;
  private WPI_TalonFX indexerB;
  private WPI_TalonFX indexerC;
  private WPI_TalonFX indexerD;
  private DigitalInput collectorSensor;
  private DigitalInput indexerSensor;
  private DigitalInput shooterSensor;

  /** Creates a new Collector. */
  public Collector() {
    collect = new WPI_TalonFX(4);
    doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1 );
    collectorSensor = new DigitalInput(0);
    indexerSensor = new DigitalInput(1);
    shooterSensor = new DigitalInput(2);
  }

  public void indexerOn() {
    indexerA.set(ControlMode.PercentOutput, 0.5);
    indexerB.set(ControlMode.PercentOutput, 0.5);
    indexerC.set(ControlMode.PercentOutput, 0.5);
    indexerD.set(ControlMode.PercentOutput, 0.5);
  }

  public void indexerOff() {
    indexerA.set(ControlMode.PercentOutput, 0);
    indexerB.set(ControlMode.PercentOutput, 0);
    indexerC.set(ControlMode.PercentOutput, 0);
    indexerD.set(ControlMode.PercentOutput, 0);
  }
  
  //This is a helper method that clarifies what winching up means in the context of the set method; 
  //might need to be inverted depending on motor orientation
  public void collectForward() {
    set(0.8);
  }

  //Might need to be inverted depending on motor orientation
  public void collectReverse() {
    set(-0.8);
  }

  //Helper method to stop the climber
  public void stop() {
    set(0);
  }

  //Main set method that can be called externally
  public void set(double speed) {
    collect.set(speed);
  }

  public void set(DoubleSolenoid.Value value) {
    doubleSolenoid1.set(value);
  }

  public DoubleSolenoid.Value getSolenoid1State() {
    return doubleSolenoid1.get();
  }

  public void toggle(){
    doubleSolenoid1.toggle();
  }

  public boolean getCollectorSensor(){
    return collectorSensor.get();
  }

  public boolean getShooterSensor(){
    return shooterSensor.get();
  }

  public boolean getIndexerSensor(){
    return indexerSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
