/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Conveyor extends SubsystemBase {
  /**
   * Creates a new TurretConveyor.
   */
  
  // conveyor includes a 'master' and a 'slave' motor/motor controller to control each side of rollers
  private TalonSRX m_master, m_slave;

  public Conveyor() {
    m_master = new TalonSRX(CONVEYOR_BACK_MOTOR);
    m_slave = new TalonSRX(CONVEYOR_FRONT_MOTOR);

    m_master.configFactoryDefault();
    m_slave.configFactoryDefault();

    // Invert the 'master' motor controller to that positive input causes an upward motion of the rollers
    m_master.setInverted(true);

    // Have the 'slave' motor controller output the same current as the 'master'
    m_slave.follow(m_master);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double output) {
    m_master.set(ControlMode.PercentOutput, output);
  }

  public void stop() {
    m_master.set(ControlMode.PercentOutput, 0.0);
  }
}
