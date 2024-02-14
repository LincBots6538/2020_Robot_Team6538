/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TimedForwardAndShoot extends SequentialCommandGroup {
  /**
   * Creates a new TimedShootAndReverse.
   */
  public TimedForwardAndShoot(Drivetrain drivetrain_subsystem, Shooter shooter_subsystem, Conveyor conveyor_subsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new SetShooterHood(shooter_subsystem, true),
          new DriveDistance(DRIVETRAIN_BASELINE_DRIVE_DISTANCE, drivetrain_subsystem),
          new TimedAutonomousThreeBallShoot(shooter_subsystem, conveyor_subsystem));
  }
}
