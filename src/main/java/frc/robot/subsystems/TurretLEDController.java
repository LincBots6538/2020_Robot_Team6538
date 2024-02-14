/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class TurretLEDController extends SubsystemBase {

    private Spark blinkin;

    public TurretLEDController() {
        blinkin = new Spark(TURRET_LED_CONTROLLER_PWM_PORT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setColor(double colorValue) {
        blinkin.set(colorValue);
    }

    public void turnOff() {
        blinkin.set(LED_COLOR_OFF);
    }
}
