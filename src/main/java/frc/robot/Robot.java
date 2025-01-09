// Copyright (c) Ada Tessar
package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This class is used to control an underwater robot with 8 thrusters (4 on each side) configured
 * for full 3D movement. The robot moves based on Xbox controller inputs.
 */
public class Robot extends TimedRobot {
    private final PWMSparkMax m_leftFrontVertical = new PWMSparkMax(0); // Left front vertical thruster
    private final PWMSparkMax m_leftRearVertical = new PWMSparkMax(1); // Left rear vertical thruster
    private final PWMSparkMax m_rightFrontVertical = new PWMSparkMax(2); // Right front vertical thruster
    private final PWMSparkMax m_rightRearVertical = new PWMSparkMax(3); // Right rear vertical thruster
    private final PWMSparkMax m_leftFrontHorizontal = new PWMSparkMax(4); // Left front horizontal thruster
    private final PWMSparkMax m_leftRearHorizontal = new PWMSparkMax(5); // Left rear horizontal thruster
    private final PWMSparkMax m_rightFrontHorizontal = new PWMSparkMax(6); // Right front horizontal thruster
    private final PWMSparkMax m_rightRearHorizontal = new PWMSparkMax(7); // Right rear horizontal thruster

    private final XboxController m_controller = new XboxController(0);

    public Robot() {
        // Set up thrusters in the SendableRegistry for debugging if needed (I will need it)
        SendableRegistry.addChild(m_leftFrontVertical, "LeftFrontVertical");
        SendableRegistry.addChild(m_leftRearVertical, "LeftRearVertical");
        SendableRegistry.addChild(m_rightFrontVertical, "RightFrontVertical");
        SendableRegistry.addChild(m_rightRearVertical, "RightRearVertical");
        SendableRegistry.addChild(m_leftFrontHorizontal, "LeftFrontHorizontal");
        SendableRegistry.addChild(m_leftRearHorizontal, "LeftRearHorizontal");
        SendableRegistry.addChild(m_rightFrontHorizontal, "RightFrontHorizontal");
        SendableRegistry.addChild(m_rightRearHorizontal, "RightRearHorizontal");
    }

    @Override
    public void teleopPeriodic() {
        // Get input values from the controller
        double x = -m_controller.getLeftY(); // Forward/backward (±x)
        double y = -m_controller.getRightX(); // Left/right (±y)
        double z = -m_controller.getRightY(); // Up/down (±z)
        double rotate = m_controller.getLeftX(); // Rotation

        // Calculate power for vertical thrusters for up/down and rotation
        double leftFrontVerticalPower = z + rotate;
        double leftRearVerticalPower = z + rotate;
        double rightFrontVerticalPower = z - rotate;
        double rightRearVerticalPower = z - rotate;

        // Calculate power for horizontal thrusters for forward/backward and strafing
        double leftFrontHorizontalPower = x + y;
        double leftRearHorizontalPower = x - y;
        double rightFrontHorizontalPower = x - y;
        double rightRearHorizontalPower = x + y;

        // Set power to thrusters
        m_leftFrontVertical.set(leftFrontVerticalPower);
        m_leftRearVertical.set(leftRearVerticalPower);
        m_rightFrontVertical.set(rightFrontVerticalPower);
        m_rightRearVertical.set(rightRearVerticalPower);

        m_leftFrontHorizontal.set(leftFrontHorizontalPower);
        m_leftRearHorizontal.set(leftRearHorizontalPower);
        m_rightFrontHorizontal.set(rightFrontHorizontalPower);
        m_rightRearHorizontal.set(rightRearHorizontalPower);
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}