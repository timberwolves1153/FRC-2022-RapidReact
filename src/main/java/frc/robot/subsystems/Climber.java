// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private WPI_TalonFX winchLeft;
  private WPI_TalonFX winchRight;

  private DoubleSolenoid doubleSolenoid1;
  private DoubleSolenoid doubleSolenoid2;

  private DigitalInput leftMagnetSensor;
  private DigitalInput rightMagnetSensor;
  private DigitalInput leftLimitSwitch;
  private DigitalInput rightLimitSwitch;

  private Accelerometer accelerometer;
  private LinearFilter xAccelFilter; 
  private LinearFilter yAccelFilter;
  private LinearFilter zAccelFilter;


  /** Creates a new Climber. */
  public Climber() {
    winchLeft =  new WPI_TalonFX(30);
    winchRight =  new WPI_TalonFX(31);
    
    doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    leftMagnetSensor = new DigitalInput(2);
    rightMagnetSensor = new DigitalInput(3);
    leftLimitSwitch = new DigitalInput(4);
    rightLimitSwitch = new DigitalInput(5);

    accelerometer = new BuiltInAccelerometer();
    xAccelFilter = LinearFilter.movingAverage(10);
    yAccelFilter = LinearFilter.movingAverage(10);
    zAccelFilter = LinearFilter.movingAverage(10);
  }

  public void updateShuffleboard(){
    SmartDashboard.putBoolean("rightWinch", getRightMagnetSensorValue());
    SmartDashboard.putBoolean("leftWinch", getLeftMagnetSensorValue());
    SmartDashboard.putNumber("rightEncoder", getRightEncoder());
    SmartDashboard.putNumber("leftEncoder", getLeftEncoder());
    SmartDashboard.putNumber("Accelerometer X", getAccelerationX());
    SmartDashboard.putNumber("Accelerometer Y", getAccelerationY());
    SmartDashboard.putNumber("Accelerometer Z", getAccelerationZ());
  }

  public void config() {
    winchRight.configFactoryDefault();
    winchLeft.configFactoryDefault();

    winchRight.setNeutralMode(NeutralMode.Brake);
    winchLeft.setNeutralMode(NeutralMode.Brake);

    winchLeft.follow(winchRight);
    winchLeft.setInverted(InvertType.InvertMotorOutput);
    winchLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    winchRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    winchRight.configNominalOutputForward(0, 100);
		winchRight.configNominalOutputReverse(0, 100);
		winchRight.configPeakOutputForward(1, 100);
		winchRight.configPeakOutputReverse(-1, 100);


    winchLeft.configNominalOutputForward(0, 100);
		winchLeft.configNominalOutputReverse(0, 100);
		winchLeft.configPeakOutputForward(1, 100);
		winchLeft.configPeakOutputReverse(-1, 100);


    winchRight.configAllowableClosedloopError(0, 0, 100);
    winchLeft.configAllowableClosedloopError(0, 0, 100);


    winchLeft.config_kF(0, 0, 100);
		winchLeft.config_kP(0, .15, 100);
		winchLeft.config_kI(0, 0, 100);
		winchLeft.config_kD(0, 1, 100);


    winchRight.config_kF(0, 0, 100);
		winchRight.config_kP(0,.15, 100);
		winchRight.config_kI(0, 0, 100);
		winchRight.config_kD(0, 1, 100);

  }

  public boolean getRightMagnetSensorValue() {
    return rightMagnetSensor.get();
  }

  public boolean getLeftMagnetSensorValue() {
    return leftMagnetSensor.get();
  }

  public double getAccelerationX() {
    return xAccelFilter.calculate(accelerometer.getX());
  }

  public double getAccelerationY() {
    return yAccelFilter.calculate(accelerometer.getY());
  }

  public double getAccelerationZ() {
    return zAccelFilter.calculate(accelerometer.getZ());
  }

  public boolean getLeftLimitSwitch(){
    return leftLimitSwitch.get();
  }

  public boolean getRightLimitSwitch(){
    return rightLimitSwitch.get();
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

  public void toggleSolenoid(){
    if(doubleSolenoid1.get().equals(DoubleSolenoid.Value.kOff) || doubleSolenoid2.get().equals(DoubleSolenoid.Value.kOff)) {
      doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
      doubleSolenoid2.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid1.toggle();
      doubleSolenoid2.toggle();
    }
  }

  public void pistonReverse(){
    doubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
    doubleSolenoid2.set(DoubleSolenoid.Value.kReverse);

  }

  public void pistonForward(){
    doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
    doubleSolenoid2.set(DoubleSolenoid.Value.kForward);
  }

  //This is a helper method that clarifies what winching up means in the context of the set method; 
  //might need to be inverted depending on motor orientation
  public void winchUp() {
    set(0.8);
  }

  //Might need to be inverted depending on motor orientation
  public void winchDown() {
    if (!leftMagnetSensor.get()) { 
      resetLeftEncoders();        // true when see black tape
      winchLeft.set(0);
    }
    else {
      winchLeft.set(-0.8);
    }

    if (!rightMagnetSensor.get()) { // true when see black tape
      resetRightEncoders();
      winchRight.set(0);
    }
    else {
      winchRight.set(-0.8);
    }

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
    winchRight.set(ControlMode.PercentOutput, speed);
    winchLeft.set(ControlMode.PercentOutput, speed);
  }

  public void setRight(double speed) {
    winchRight.set(speed);
  }

  public void setLeft(double speed) {
    winchLeft.set(speed);
  }

  public void resetRightEncoders(){
    winchRight.setSelectedSensorPosition(0, 0, 100);
   
  }

  public void resetLeftEncoders(){
    winchLeft.setSelectedSensorPosition(0, 0, 100);
  }

  public void climbToSetPoint(int setPoint){
    winchLeft.set(ControlMode.Position, setPoint);
    winchRight.set(ControlMode.Position, setPoint);
  }


  public double getRightEncoder(){
   return winchRight.getSelectedSensorPosition();
  }
  
  public double getLeftEncoder(){
    return winchLeft.getSelectedSensorPosition();
  }

  public double getEncoderTicksFromPosition(double distance) {
    return (2048 * 15) / (0.5 * Math.PI) * distance;
  }

  public void setWinchPosition(double encoderTicks){
    winchLeft.set(TalonFXControlMode.Position, encoderTicks);
    winchRight.set(TalonFXControlMode.Position, encoderTicks);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}