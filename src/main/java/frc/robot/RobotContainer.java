/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ActivateConveyor;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.Agitate;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.VelocityPIDTeleopDrive;
import frc.robot.commands.IntelligentVisionControl;
import frc.robot.commands.ManualAdjustShooterVelocity;
import frc.robot.commands.ManualAdjustTurret;
import frc.robot.commands.ManualControlConveyor;
import frc.robot.commands.ManualDropShooterHood;
import frc.robot.commands.ManualSpinIntakeRollers;
import frc.robot.commands.SetLEDStripColor;
import frc.robot.commands.TestShooterHood;
import frc.robot.commands.ThreeBallAutonShoot;
import frc.robot.commands.TimedForwardAndShoot;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretLEDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Miscellaneous
  private SendableChooser autonChooser;
  private final Limelight limelight = new Limelight("limelight");
  private final TurretLEDController blinkin = new TurretLEDController();
  private final Compressor compressor = new Compressor(COMPRESSOR_ID);
  private final GenericHID usbcontroller0 = new XboxController(0);
  private final GenericHID usbcontroller1 = new XboxController(1);
  private final GenericHID[] controllers = {usbcontroller0, usbcontroller1};

  // Subsystems
  private final Drivetrain rDrivetrain = new Drivetrain();
  private final Intake rFrontIntake = new Intake(FRONT_INTAKE_MOTOR, true, PCM_ID,
    FRONT_INTAKE_DSOL_FORWARD_CHANNEL, FRONT_INTAKE_DSOL_REVERSE_CHANNEL);
  private final Intake rRearIntake = new Intake(REAR_INTAKE_MOTOR, true, PCM_ID,
    REAR_INTAKE_DSOL_FORWARD_CHANNEL, REAR_INTAKE_DSOL_REVERSE_CHANNEL);
  private final Agitator rAgitator = new Agitator();
  private final Turret rTurret = new Turret();
  private final Conveyor rConveyor = new Conveyor();
  private final Shooter rShooter = new Shooter();
  
  // Commands
  private final DriveDistance AutonBaselineDriveCommand = new DriveDistance(DRIVETRAIN_BASELINE_DRIVE_DISTANCE, rDrivetrain);
  private final ThreeBallAutonShoot AutonBasicShootCommand = new ThreeBallAutonShoot(rDrivetrain, rTurret, rShooter, rConveyor, limelight, blinkin);
  private final TimedForwardAndShoot TimedForwardAndShootCommand = new TimedForwardAndShoot(rDrivetrain, rShooter, rConveyor);
  private final VelocityPIDTeleopDrive VelocityPIDTeleopDriveCommand = new VelocityPIDTeleopDrive(rDrivetrain);
  private final IntelligentVisionControl intelligentVisionControlCommand = new IntelligentVisionControl(false, rTurret, rShooter, rConveyor, limelight, blinkin);
  private final SetLEDStripColor StandbyLEDsCommand = new SetLEDStripColor(blinkin, LED_COLOR_BLUE);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set the Compressor to be in closed loop control mode (default)
    compressor.setClosedLoopControl(true);

    rDrivetrain.setDefaultCommand(VelocityPIDTeleopDriveCommand);
    rDrivetrain.resetEncoders();

    rFrontIntake.setSolenoid(Value.kReverse);
    rRearIntake.setSolenoid(Value.kReverse);
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure the 'Control Panel' of the Shuffleboard
    configureControlPanel();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton aButton_p0 = new JoystickButton(getController(0), 1);
    //JoystickButton bButton_p0 = new JoystickButton(getController(0), 2);
    //JoystickButton xButton_p0 = new JoystickButton(getController(0), 3);
    //JoystickButton yButton_p0 = new JoystickButton(getController(0), 4);
    JoystickButton lbButton_p0 = new JoystickButton(getController(0), 5);
    JoystickButton rbButton_p0 = new JoystickButton(getController(0), 6);

    JoystickButton xButton_p1 = new JoystickButton(getController(1), 3);
    JoystickButton yButton_p1 = new JoystickButton(getController(1), 4);
    JoystickButton lbButton_p1 = new JoystickButton(getController(1), 5);
    JoystickButton rbButton_p1 = new JoystickButton(getController(1), 6);
    JoystickButton lsButton_p1 = new JoystickButton(getController(1), 9);
    POVButton degree0Button_p1 = new POVButton(getController(1), 0);
    POVButton degree180Button_p1 = new POVButton(getController(1), 180);
    BooleanSupplier bs1_p1 = () -> Math.abs(getController(1).getX(Hand.kRight)) >= 0.25;
    Trigger rightXAxisTrigger_p1 = new Trigger(bs1_p1);
    BooleanSupplier bs2_p1 = () -> getController(1).getRawAxis(2) >= 0.25 ||
      getController(1).getRawAxis(3) >= 0.25;
    Trigger combinedTriggersTrigger_p1 = new Trigger(bs2_p1);
    JoystickButton startButton_p1 = new JoystickButton(getController(1), 8);
    
    aButton_p0.toggleWhenPressed(new ActivateConveyor(rConveyor));
    //xButton_p0.whileHeld(new Agitate(rAgitator, false));
    //yButton_p0.whileHeld(new Agitate(rAgitator, true));
    lbButton_p0.whileHeld(new ActuateIntake(rRearIntake));
    rbButton_p0.whileHeld(new ActuateIntake(rFrontIntake));

    xButton_p1.toggleWhenPressed(new ManualDropShooterHood(rShooter));
    yButton_p1.toggleWhenPressed(new ManualSpinIntakeRollers(rFrontIntake, true))
              .toggleWhenPressed(new ManualSpinIntakeRollers(rRearIntake, true));
    lbButton_p1.toggleWhenPressed(new ActuateIntake(rRearIntake));
    rbButton_p1.toggleWhenPressed(new ActuateIntake(rFrontIntake));
    lsButton_p1.toggleWhenPressed(intelligentVisionControlCommand);
    degree0Button_p1.whenPressed(new ManualAdjustShooterVelocity(rShooter, true));
    degree180Button_p1.whenPressed(new ManualAdjustShooterVelocity(rShooter, false));
    rightXAxisTrigger_p1.whileActiveOnce(new ManualAdjustTurret(rTurret));
    combinedTriggersTrigger_p1.whileActiveOnce(new ManualControlConveyor(rConveyor));
    startButton_p1.toggleWhenPressed(new ManualSpinIntakeRollers(rFrontIntake, false))
                  .toggleWhenPressed(new ManualSpinIntakeRollers(rRearIntake, false));
  }

  private void configureControlPanel() {
    /**
     * Options for Autonomous mode.
     */
    autonChooser = new SendableChooser();
    autonChooser.setDefaultOption("Drive Past Baseline", AutonBaselineDriveCommand);
    autonChooser.addOption("3 Ball Autonomous Shoot", AutonBasicShootCommand);
    autonChooser.addOption("Timed 3 Ball Autonomous Shoot", TimedForwardAndShootCommand);
    autonChooser.addOption("No Autonomous", null);

    /**
     * Create a new Tab on the Shuffleboard to act as a 'Control Panel' for the drivers.
     */
    ShuffleboardTab tab = Shuffleboard.getTab("Control Panel");
    tab.add("Autonomous Selector", autonChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public GenericHID getController(int controller_index) {
    return controllers[controller_index];
  }

  public void putValuesToShuffleboard() {
    limelight.putValuesToShuffleboard();
    rDrivetrain.putValuesToShuffleboard();
    rShooter.putValuesToShuffleboard();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return (Command) autonChooser.getSelected();
  }
}
