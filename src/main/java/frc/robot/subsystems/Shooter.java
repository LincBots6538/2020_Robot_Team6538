/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private TalonSRX m_master, m_slave;
  private Solenoid solenoid;
  private Encoder encoder;
  private PIDController pid;
  private Timer timer;
  private int enc_pos_init, enc_pos_fin;
  private double time_init, time_fin;

  /**
   * Create a new Tab on the Shuffleboard for drivetrain information.
   */
  private ShuffleboardTab tab_shooter = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry percentOutputEntry = tab_shooter.add("Percent-Output", 0).getEntry();
  private NetworkTableEntry velocityEntry = tab_shooter.add("Velocity", 0).getEntry();

  public Shooter() {
    //Left and right motors connected to shooter
    m_master = new TalonSRX(SHOOTER_LEFT_MOTOR);
    m_slave = new TalonSRX(SHOOTER_RIGHT_MOTOR);

    m_master.configFactoryDefault();
    m_slave.configFactoryDefault();

    // Invert both motors so that a positive input results in an outward spin of the flywheel
    m_master.setInverted(true);
    m_slave.setInverted(true);
    
    /*
    // Configure the PID gains of the 'master' motor controller
    m_master.config_kP(SHOOTER_kPIDLoopIdx, SHOOTER_kGains.kP);
    m_master.config_kI(SHOOTER_kPIDLoopIdx, SHOOTER_kGains.kI);
    m_master.config_kD(SHOOTER_kPIDLoopIdx, SHOOTER_kGains.kD);
    */

    // Configure the 'slave' motor controller to output the same current as the 'master'
    m_slave.follow(m_master);

    // The solenoid attached to the shooter's hood
    solenoid = new Solenoid(PCM_ID, SHOOTER_SOL_CHANNEL);

    // The encoder attached to the shooter's motor
    encoder = new Encoder(SHOOTER_ENCODER_CHANNEL_A, SHOOTER_ENCODER_CHANNEL_B);

    // Set up the PID controller used to control the velocity of the shooter
    pid = new PIDController(SHOOTER_kGains.kP, SHOOTER_kGains.kI, SHOOTER_kGains.kD);

    timer = new Timer();
    timer.reset();
    resetEncoder();
    enc_pos_init = getEncoderPosition();
    time_init = timer.get();
    timer.start();
  }

  public int getEncoderPosition() {
    return encoder.getRaw();
  }

  public double getPercentOutput() {
    return m_master.getMotorOutputPercent();
  }

  public double getPIDOutput(double measurement) {
    return pid.calculate(measurement);
  }

  public double getPIDSetpoint() {
    return pid.getSetpoint();
  }

  public boolean getSolenoid() {
    return solenoid.get();
  }

  public double getVelocity() {
    // Returns raw sensor units per 100ms
    return (enc_pos_fin - enc_pos_init) / (time_fin - time_init);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    time_fin = timer.get() * 1000; // time in ms
    if(time_fin - time_init >= 100)
    {
      enc_pos_fin = getEncoderPosition();
      getVelocity();
      enc_pos_init = getEncoderPosition();
      time_init = time_fin;
    }
  }

  // Updates the values on the Shuffleboard
  public void putValuesToShuffleboard() {
    // Percent-output of shooter motors
    percentOutputEntry.setDouble(getPercentOutput());
    // Velocity of the shooter motors
    velocityEntry.setDouble(getVelocity());
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public void set(double output) {
    m_master.set(ControlMode.PercentOutput, output);
  }

  public void setPIDSetpoint(double setpoint) {
    pid.setSetpoint(setpoint);
  }

  public void setSolenoid(boolean state) {
    solenoid.set(state);
  }

  public void stop(){
    m_master.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean withinVelocityTolerance(int desired_velocity) {
    if(getVelocity() >= desired_velocity - SHOOTER_VELOCITY_TOLERANCE
      && getVelocity() <= desired_velocity + SHOOTER_VELOCITY_TOLERANCE)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
