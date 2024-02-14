/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

public class TimedAutonomousThreeBallShoot extends CommandBase {
  /**
   * Creates a new TimedAutonomousThreeBallShoot.
   */

  Shooter shooter_subsystem;
  Conveyor conveyor_subsystem;
  Timer timer;

  public TimedAutonomousThreeBallShoot(Shooter shooter_subsystem, Conveyor conveyor_subsystem) {
    this.shooter_subsystem = shooter_subsystem;
    this.conveyor_subsystem = conveyor_subsystem;
    timer = new Timer();
    addRequirements(shooter_subsystem, conveyor_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    shooter_subsystem.set(0.8);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > 4)
    {
      conveyor_subsystem.set(CONVEYOR_SPEED);
    }
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
    return timer.get() > 6;
  }
}
