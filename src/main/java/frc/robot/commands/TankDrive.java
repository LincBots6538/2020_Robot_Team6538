/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  /**
   * Creates a new TankDrive.
   */
  private Drivetrain drivetrain_subsystem;
  
  private GenericHID controller;

  private double leftVelocity, rightVelocity;
  
  public TankDrive(Drivetrain subsystem) {
    drivetrain_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = Robot.m_robotContainer.getController(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftVelocity = controller.getY(GenericHID.Hand.kLeft) * -1;
    rightVelocity = controller.getY(GenericHID.Hand.kRight) * -1;

    drivetrain_subsystem.setLeftVelocity(leftVelocity);
    drivetrain_subsystem.setRightVelocity(rightVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
