package frc.robot.platforms;

import frc.robot.utilities.Util;

/**
 * RobotPlatformBase class represents the minimum required methods needed
 * to be executed in the RobotContainer.
 * 
 * Each robot configuration can be represented as a class which implements
 * RobotPlatform. The class implementation shall contain all of the state
 * for its components and dependent systems.
 */
public class RobotPlatformBase {
    protected void log(String s) { Util.log(s); }
    protected void logf(String pattern, Object... arguments) { Util.logf(pattern, arguments); }

    public void robotInit() {};
    public void robotPeriodic() {};

    public void driverStationConnected() {};

    public void simulationInit() {};
    public void simulationPeriodic() {};

    public void disabledInit() {};
    public void disabledPeriodic() {};
    public void disabledExit() {};

    public void autonomousInit() {};
    public void autonomousPeriodic() {};
    public void autonomousExit() {};

    public void teleopInit() {};
    public void teleopPeriodic() {};
    public void teleopExit() {};

    public void testInit() {};
    public void testPeriodic() {};
    public void testExit() {};
}


