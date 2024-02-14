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
import frc.robot.subsystems.Drivetrain;

public class PIDTeleopDrive extends CommandBase {
  /**
   * Creates a new PIDTeleopDrive.
   */
  private Drivetrain drivetrain_subsystem;

  private XboxController controller;
  private double leftTriggerVal, rightTriggerVal;

  private double currentLeftMeasurement, currentRightMeasurement;
  private double newLeftTarget, newRightTarget;
  
  public PIDTeleopDrive(Drivetrain subsystem) {
    drivetrain_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = (XboxController) Robot.m_robotContainer.getController(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    currentLeftTarget = drivetrain_subsystem.getLeftTargetMeasurement();
    currentRightTarget = drivetrain_subsystem.getRightTargetMeasurement();
    */
    currentLeftMeasurement = drivetrain_subsystem.getLeftEncoderMeasurement();
    currentRightMeasurement = drivetrain_subsystem.getRightEncoderMeasurement();

    leftTriggerVal = controller.getTriggerAxis(Hand.kLeft) * -1;
    rightTriggerVal = controller.getTriggerAxis(Hand.kRight);

    if(Math.abs(leftTriggerVal) < 0.1) { leftTriggerVal = 0.0; }
    if(Math.abs(rightTriggerVal) < 0.1) { rightTriggerVal = 0.0; }

    double target_manipulator = (leftTriggerVal + rightTriggerVal) * 12;

    newLeftTarget = currentLeftMeasurement + target_manipulator;
    newRightTarget = currentRightMeasurement + target_manipulator;

    drivetrain_subsystem.setLeftTarget(newLeftTarget);
    drivetrain_subsystem.setRightTarget(newRightTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain_subsystem.PIDStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
