package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class BlueRoboticsBasicESC extends PWMMotorController{
     /**
   * Constructor.
   *
   * @param channel The PWM channel that the BlueRobotics Basic ESC is attached to. 0-9 are on-board, 10-19
   *     are on the MXP port
   */
  @SuppressWarnings("this-escape")
  public BlueRoboticsBasicESC(final int channel) {
    super("BlueRoboticsBasicESC", channel);

    m_pwm.setBoundsMicroseconds(1900, 1550, 1500, 1450, 1100);
    m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
    m_pwm.setSpeed(0.0);
    m_pwm.setZeroLatch();
  }
}
