package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class BlueRoboticsNewton extends PWMMotorController{
     /**
   * Constructor.
   *
   * @param channel The PWM channel that the BlueRobotics Basic ESC is attached to. 0-9 are on-board, 10-19
   *     are on the MXP port
   */
  @SuppressWarnings("this-escape")
  public BlueRoboticsNewton(final int channel) {
    super("BlueRoboticsNewton", channel);

    m_pwm.setBoundsMicroseconds(1920, 1550, 1500, 1450, 1050);
    m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
    m_pwm.setSpeed(0.0);
    m_pwm.setZeroLatch();
  }
}
