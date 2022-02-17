// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  public enum ShooterPosition {
    AUTO_LINE(0), CR_CLOSE(1), DOWNTOWN(2), INVALID(3);

    private int value;
    
    private ShooterPosition(int value) {
      this.value = value;
    }

    public int getPosition() {
      return value;
    }

    public static ShooterPosition fromInt(int value) {
      for(ShooterPosition position : values()) {
        if(position.value == value) {
          return position;
        }
      }
      return INVALID;
    }

    public static int getHighestValue() {
      int highestVal = 0;
      for(ShooterPosition position : values()) {
        if(position.value > highestVal) {
          highestVal = position.value;
        }
      }
      return highestVal - 1;
    }
  }

  public enum Direction {
    kForwards(1), kBackwards(-1);

    private int direction;

    private Direction(int direction) {
      this.direction = direction;
    }

    public int getDirection() {
      return direction;
    }
  }

  private WPI_TalonFX bottomRoller;
  private WPI_TalonFX topRoller;
  private CANSparkMax feeder;


  private SparkMaxPIDController launcherPID;

  private boolean pidEnabled = false;

  private static final double[] SHOOTER_P = {0.0000001, 0.0000001, 0.0000001};
  private static final double[] SHOOTER_F = {(.6/3400), (.75/3900), (.82/4500)};
  private static final double[] SHOOTER_SETPOINT = {3400, 3700, 4500};
  //Setpoint Values: 3400, 4100, 4500

  private double p, i, d, f, setpoint;

  private ShooterPosition defaultPosition = ShooterPosition.AUTO_LINE;
  private ShooterPosition selectedPosition;

  private static final double MAX_OUTPUT = 1;
  private static final double MIN_OUTPUT = -1;

  /** Creates a new Shooter. */
  public Launcher() {
    feeder = new CANSparkMax(20, MotorType.kBrushless);
    bottomRoller = new WPI_TalonFX(21);
    topRoller = new WPI_TalonFX(22);

    config();
  }

  public void config() {
    bottomRoller.configFactoryDefault();

    bottomRoller.setNeutralMode(NeutralMode.Coast);

    bottomRoller.setInverted(true);

    bottomRoller.configNeutralDeadband(0.001);

    bottomRoller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    bottomRoller.configPeakOutputForward(MAX_OUTPUT);
    bottomRoller.configPeakOutputReverse(MIN_OUTPUT);

    bottomRoller.configNominalOutputForward(0);
    bottomRoller.configNominalOutputReverse(0);


    topRoller.configFactoryDefault();

    topRoller.setNeutralMode(NeutralMode.Coast);

    topRoller.setInverted(true);

    topRoller.configNeutralDeadband(0.001);

    topRoller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    topRoller.configPeakOutputForward(MAX_OUTPUT);
    topRoller.configPeakOutputReverse(MIN_OUTPUT);

    topRoller.configNominalOutputForward(0);
    topRoller.configNominalOutputReverse(0);


  }

  public void updateShuffleboard() {
    SmartDashboard.putNumber("Shooter Velocity", bottomRoller.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Power", bottomRoller.get());
    SmartDashboard.putNumber("Shooter Position", defaultPosition.getPosition());

    double p = SmartDashboard.getNumber("Shooter P", this.p);
    double i = SmartDashboard.getNumber("Shooter I", this.i);
    double d = SmartDashboard.getNumber("Shooter D", this.d);
    double f = SmartDashboard.getNumber("Shooter F", this.f);
    double setpoint = SmartDashboard.getNumber("Shooter Setpoint", this.setpoint);

    SmartDashboard.putString("Shooter Position", selectedPosition.getClass().getName());

    // If there are any changes from Shuffleboard, update the PID Controller
    if (this.p != p || this.i != i || this.d != d || this.f != f || this.setpoint != setpoint) {
      setPIDGains(p, i, d, f, setpoint);
      System.out.println("Updating Values");
    }
  }

  public void setSpeed(double speed) {
    bottomRoller.set(-speed);
  }

  public void stop() {
    bottomRoller.set(0);
  }

  /**
   * Convenience wrapper that sets PID parameters according to pre-stored values that
   * correspond with the given shooter position
   * @param shooterPosition Indicates the shooter position for which PID values will be updated
   */
  public void setGainPreset(ShooterPosition shooterPosition) {
    int pos = shooterPosition.getPosition();
    selectedPosition = shooterPosition;

    // I and D values are not pulled from an array since these values are always zero
    setPIDGains(SHOOTER_P[pos], 0, 0, SHOOTER_F[pos], SHOOTER_SETPOINT[pos]);
  }

  /**
   * Sets all tuning parameters for the rotation-regulating PID
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param f Feed-forward gain
   * @param setpoint RPM setpoint
   */
  public void setPIDGains(double p, double i, double d, double f, double setpoint) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    this.setpoint = setpoint;

    launcherPID.setP(p);
    launcherPID.setI(i);
    launcherPID.setD(d);
    launcherPID.setFF(f);

    SmartDashboard.putNumber("Shooter P Gain", p);
    SmartDashboard.putNumber("Shooter I Gain", i);
    SmartDashboard.putNumber("Shooter D Gain", d);
    SmartDashboard.putNumber("Shooter F Gain", f);
    SmartDashboard.putNumber("Shooter Setpoint", setpoint);
  }

  /**
   * Convenience method that sets PID parameters according to pre-stored default values
   */
  public void resetGainPreset() {
    setGainPreset(defaultPosition);
  }

  /**
   * Sets a flag that will signal the PID to run periodically
   */
  public void pidOn() {
    if (!pidEnabled) {
      pidEnabled = true;
      launcherPID.setIAccum(0);
    }
  }

  /**
   * Runs all periodic logic for the PID controller, called periodically while
   * the PID is enabled
   */
  private void pidPeriodic() {
    launcherPID.setReference(setpoint, ControlType.kVelocity);
  }

  public void cycleGainPreset(Direction direction) {
    int highestVal = ShooterPosition.getHighestValue();
    int nextPosition = defaultPosition.getPosition() + direction.getDirection();
    if(nextPosition < 0) {
      defaultPosition = ShooterPosition.fromInt(highestVal);
    } else if(nextPosition > highestVal) {
      defaultPosition = ShooterPosition.fromInt(0);
    } else {
      defaultPosition = ShooterPosition.fromInt(nextPosition);
    }
    setGainPreset(defaultPosition);
  }

  public double getShooterVelocity() {
    return bottomRoller.getSelectedSensorVelocity();
  }

  public double getSetPoint() {
    return SHOOTER_SETPOINT[selectedPosition.getPosition()];
  }

  public boolean isAtSetpoint() {
    return bottomRoller.getSelectedSensorVelocity() >= getSetPoint() - 50 || bottomRoller.getSelectedSensorVelocity() <= getSetPoint() + 50;
  }

  public ShooterPosition getSelectedPosition() {
    return selectedPosition;
  }

  /**
   * Sets a flag that will prevent the PID from running periodically
   */
  public void pidOff() {
    pidEnabled = false;
    bottomRoller.set(0);
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}