/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static  frc.robot.Constants.*;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  private TalonSRX m_talon;
  private PIDController pid;
  
  public Turret() {
    m_talon = new TalonSRX(TURRET_MOTOR);
    
    m_talon.configFactoryDefault();
    m_talon.setInverted(true);

    m_talon.configPeakOutputForward(TURRET_PEAK_OUTPUT);
    m_talon.configPeakOutputReverse(-TURRET_PEAK_OUTPUT);
    
    pid = new PIDController(TURRET_kGains.kP, TURRET_kGains.kI, TURRET_kGains.kD);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    m_talon.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    m_talon.set(ControlMode.PercentOutput, 0.0);
  }

  public double getPIDSetpoint() {
    return pid.getSetpoint();
  }

  public void setPIDSetpoint(double setpoint) {
    pid.setSetpoint(setpoint);
  }

  public double getPIDOutput(double measurement) {
    return pid.calculate(measurement);
  }
}
