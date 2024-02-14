/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class DriveDistance extends CommandBase {
  /**
   * Creates a new DriveDistance.
   */

  private Drivetrain drivetrain_subsystem;
  private double distance_to_drive;
  private double init_left_dist, init_right_dist;
  private double left_setpoint, right_setpoint;
  private double current_left_dist, current_right_dist;
  private double tolerance = DRIVETRAIN_CLOSED_LOOP_TOLERANCE;

  public DriveDistance(double _distance, Drivetrain subsystem) {
    drivetrain_subsystem = subsystem;
    distance_to_drive = _distance;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init_left_dist = drivetrain_subsystem.getLeftEncoderMeasurement();
    init_right_dist = drivetrain_subsystem.getRightEncoderMeasurement();

    left_setpoint = init_left_dist + distance_to_drive;
    right_setpoint = init_right_dist + distance_to_drive;

    drivetrain_subsystem.setLeftTarget(left_setpoint);
    drivetrain_subsystem.setRightTarget(right_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_left_dist = drivetrain_subsystem.getLeftEncoderMeasurement();
    current_right_dist = drivetrain_subsystem.getRightEncoderMeasurement();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(current_left_dist >= left_setpoint - tolerance && current_left_dist <= left_setpoint + tolerance
        && current_right_dist >= right_setpoint - tolerance && current_right_dist <= right_setpoint + tolerance)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
