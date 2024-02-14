/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Limelight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretLEDController;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ThreeBallAutonShoot extends SequentialCommandGroup {
  /**
   * Creates a new ThreeBallAutonShoot.
   */
  public ThreeBallAutonShoot(Drivetrain rDrivetrain, Turret turret_subsystem, Shooter shooter_subsystem, Conveyor conveyor_subsystem, Limelight limelight, TurretLEDController _blinkin) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new IntelligentVisionControl(true, turret_subsystem, shooter_subsystem, conveyor_subsystem, limelight, _blinkin),
          new DriveDistance(DRIVETRAIN_BASELINE_DRIVE_DISTANCE, rDrivetrain));
  }
}
