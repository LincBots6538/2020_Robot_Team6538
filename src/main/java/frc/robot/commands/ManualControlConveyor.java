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
import frc.robot.subsystems.Conveyor;

public class ManualControlConveyor extends CommandBase {
  /**
   * Creates a new ManualControlConveyor.
   */

  private Conveyor conveyor_subsystem;
  private XboxController controller;
  private double forwardVal, reverseVal, output;

  public ManualControlConveyor(Conveyor subsystem) {
    conveyor_subsystem = subsystem;
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
    forwardVal = controller.getTriggerAxis(Hand.kRight);
    reverseVal = controller.getTriggerAxis(Hand.kLeft) * -1;
    output = forwardVal + reverseVal;
    conveyor_subsystem.set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
