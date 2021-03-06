// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  private WPI_TalonFX collect;

  private DoubleSolenoid doubleSolenoid1;
  private DoubleSolenoid doubleSolenoid2;

  private CANSparkMax mover;
  private CANSparkMax singulator;

  private DigitalInput moverBannerSensor;
  private DigitalInput feederBannerSensor;

  private CANSparkMax feeder;

  /** Creates a new Collector.*/
  public Collector() {
    collect = new WPI_TalonFX(10);
    singulator = new CANSparkMax(11, MotorType.kBrushless);
    mover = new CANSparkMax(12, MotorType.kBrushless);
    feeder = new CANSparkMax(20, MotorType.kBrushless);

    doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    // doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    // doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    moverBannerSensor = new DigitalInput(1);
    feederBannerSensor = new DigitalInput(0);
    
    mover.restoreFactoryDefaults();
    collect.configFactoryDefault();
    singulator.restoreFactoryDefaults();

    mover.setIdleMode(IdleMode.kBrake);
    singulator.setIdleMode(IdleMode.kBrake);
    collect.setNeutralMode(NeutralMode.Brake);

    mover.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    singulator.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);

    mover.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    singulator.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    mover.burnFlash();
    singulator.burnFlash();
  }

  public void config() {
    feeder.restoreFactoryDefaults();
    feeder.setIdleMode(IdleMode.kBrake);
    feeder.burnFlash();
    collect.configFactoryDefault();

    collect.setNeutralMode(NeutralMode.Brake);

    collect.setInverted(InvertType.InvertMotorOutput);

    collect.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    collect.configNominalOutputForward(0, 100);
    collect.configNominalOutputReverse(0, 100);
    collect.configPeakOutputForward(1, 100);
    collect.configPeakOutputReverse(-1, 100);

    collect.configAllowableClosedloopError(0, 0, 100);  

    collect.config_kF(0, 0, 100);
    collect.config_kP(0, 0.15, 100);
    collect.config_kI(0, 0, 100);
    collect.config_kD(0, 1, 100);
  }

  public void moverForward() {
      mover.set(0.5);
  }

  public void feederOn(){
    feeder.set(-0.8);
  }
  public void feederOff(){
    feeder.set(0);
  }

  public void moverOff() {
    mover.set(0);
  }

  public void moverReverse() {
    mover.set(-0.5);
  }
  public boolean getFeederBannerSensor() {
    return feederBannerSensor.get();
  }
  
  //This is a helper method that clarifies what winching up means in the context of the set method; 
  //might need to be inverted depending on motor orientation
  public void collectIntake() {
    collect.set(0.5);
  }

  //Might need to be inverted depending on motor orientation
  public void collectOutake() {
    collect.set(-0.5);
  }

  public void singulatorIntake() {
    singulator.set(-1);
  }
  
  public void singulatorOutake() {
    singulator.set(1);
  }

  public void singulatorOff() {
    singulator.set(0);
  }

  //Helper method to stop the climber
  public void collectorStop() {
    collect.set(0);
  }

  //Main set method that can be called externally
  public void set(double speed) {
    collect.set(speed);
  }

  public void resetRightCollectorEncoders(){
    collect.setSelectedSensorPosition(0, 0, 100);
  }
  public double getEncoderTicksFromPosition(double distanceInches) {
    return (2048 * 15) / (0.5 * Math.PI) * distanceInches;
  }

  public void climbToSetPoint(int setPoint){
    collect.set(ControlMode.Position, setPoint);
  }

  public void setWinchPosition(double encoderTicks){
    System.out.println("Setting winch to " + encoderTicks + " encoder ticks");
    collect.set(TalonFXControlMode.Position, encoderTicks);
  }

  // public void smartBallCollect(){
  //   singulator.set(-1);
  //   if(!feederSeesBall() && moverSeesBall()){
  //     feeder.set(-0.2);
  //     mover.set(0.5);
  //   }
  //   else if(feederSeesBall()&&moverSeesBall()){
  //     feeder.set(0);
  //     mover.set(0);
  //   }
  //   else if(feederSeesBall() && !moverSeesBall()){
  //     feeder.set(0);
  //     mover.set(0.5);
  //   }
  //   else{//NO BALLS IN ROBOT
  //     feeder.set(-0.2);
  //     mover.set(0.5);
  //   }
  // }

  public void setSolenoid(DoubleSolenoid.Value value) {
    doubleSolenoid1.set(value);
   // doubleSolenoid2.set(value);
  }

  public DoubleSolenoid.Value getSolenoidState() {
    return doubleSolenoid1.get();
  }

  // public DoubleSolenoid.Value getSolenoid2State() {
  //   return doubleSolenoid2.get();
  // }

  public void toggleSolenoid(){
    doubleSolenoid1.toggle();
   // doubleSolenoid2.toggle();
    if(doubleSolenoid1.get().equals(DoubleSolenoid.Value.kOff) /*&& doubleSolenoid2.get().equals(DoubleSolenoid.Value.kOff)*/) {
      setSolenoid(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getMoverBannerSensor() {
    return moverBannerSensor.get();
  }
  
  public boolean moverSeesBall(){
    return !moverBannerSensor.get();
  }

  public boolean feederSeesBall(){
    return !feederBannerSensor.get();
  }
  
  public void updateShuffleboard(){
    SmartDashboard.putBoolean("Feeder Banner Sensor", getFeederBannerSensor());
    SmartDashboard.putBoolean("Mover Banner Sensor", getMoverBannerSensor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
