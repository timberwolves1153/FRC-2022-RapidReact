// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Collector extends SubsystemBase {

  private WPI_TalonFX collect;
  private DoubleSolenoid doubleSolenoidRight;
  private DoubleSolenoid doubleSolenoidLeft;
  private CANSparkMax mover;
  private CANSparkMax singulator;
  private DigitalInput moverSensor;
  private DigitalInput singulatorSensor;
  private DigitalInput moverBannerSensor;
  private DigitalInput singulatorBannerSensor;
  

  /** Creates a new Collector.*/
  public Collector() {
    collect = new WPI_TalonFX(10);
    singulator = new CANSparkMax(11, MotorType.kBrushless);
    mover = new CANSparkMax(12, MotorType.kBrushless);

    doubleSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    doubleSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    moverSensor = new DigitalInput(0);
    singulatorSensor = new DigitalInput(1);
    moverBannerSensor = new DigitalInput(6);
    singulatorBannerSensor = new DigitalInput(7);

    mover.restoreFactoryDefaults();
    collect.configFactoryDefault();
    singulator.restoreFactoryDefaults();

    mover.setIdleMode(IdleMode.kBrake);
    singulator.setIdleMode(IdleMode.kBrake);
    collect.setNeutralMode(NeutralMode.Brake); 

    mover.burnFlash();
    singulator.burnFlash();

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
    collect.set(0.5);
  }

  //Might need to be inverted depending on motor orientation
  public void outake() {
    collect.set(-1);
  }

  public void singulatorIntake() {
    singulator.set(-0.5);
  }

  public void singulatorOutake() {
    singulator.set(0.5);
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
    doubleSolenoidRight.set(value);
    doubleSolenoidLeft.set(value);
  }

  public DoubleSolenoid.Value getRightSolenoidState() {
    return doubleSolenoidRight.get();
  }

  public DoubleSolenoid.Value getLeftSolenoidState() {
    return doubleSolenoidLeft.get();
  }

  public void toggleSolenoid(){
    doubleSolenoidRight.toggle();
    doubleSolenoidLeft.toggle();
    if(doubleSolenoidRight.get().equals(DoubleSolenoid.Value.kOff)) {
      setSolenoid(DoubleSolenoid.Value.kForward);
    }
    if(doubleSolenoidLeft.get().equals(DoubleSolenoid.Value.kOff)) {
      setSolenoid(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getMoverColorSensor(){
    return moverSensor.get();
  }
  public boolean getMoverBannerSensor() {
    return moverBannerSensor.get();
  }

  public boolean getSingulatorColorSensor(){
    return singulatorSensor.get();
  }
  public boolean getSingulatorBannerSensor() {
    return singulatorBannerSensor.get();
  }

  public void updateShuffleboard(){
    SmartDashboard.putBoolean("Singulator Banner Sensor", getSingulatorBannerSensor());
    SmartDashboard.putBoolean("Singulator Color Sensor", getSingulatorColorSensor());
    SmartDashboard.putBoolean("Mover Banner Sensor", getMoverBannerSensor());
    SmartDashboard.putBoolean("Mover Color Sensor", getMoverColorSensor());
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
