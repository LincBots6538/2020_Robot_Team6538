/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

public class TestConveyorAndShooter extends CommandBase {
  /**
   * Creates a new TestConveyorAndShooter.
   */
  private Conveyor conveyor_subsystem;
  private Shooter shooter_subsystem;
  
  public TestConveyorAndShooter(Conveyor _conveyor_subsystem, Shooter _shooter_subsystem) {
    conveyor_subsystem = _conveyor_subsystem;
    shooter_subsystem = _shooter_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_conveyor_subsystem, _shooter_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor_subsystem.set(CONVEYOR_SPEED);
    shooter_subsystem.set(SHOOTER_PEAK_OUTPUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor_subsystem.stop();
    shooter_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
