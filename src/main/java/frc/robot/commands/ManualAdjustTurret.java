/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class ManualAdjustTurret extends CommandBase {
  /**
   * Creates a new TestTurret.
   */
  private Turret turret_subsystem;
  private XboxController controller;
  private double output;

  public ManualAdjustTurret(Turret subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = (XboxController) Robot.m_robotContainer.getController(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = controller.getX(Hand.kRight)/3;
    turret_subsystem.set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
