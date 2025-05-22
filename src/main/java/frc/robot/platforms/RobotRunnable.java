package frc.robot.platforms;

import frc.robot.Robot;
import frc.robot.utilities.Util;

public interface RobotRunnable {
    public default void log(String s) { Util.logf("(%s) %s", robotName(), s); }
    public default void logf(String pattern, Object... arguments) { Util.logf("(%s) " + pattern, robotName(), arguments); }
    public default void logfPeriodic(String pattern, Object... arguments) { if (Robot.count % 10 == 0) { logf(pattern, arguments); } }
    public default String robotName() { return "robotname-undefined"; }

    public default void robotInit() { log("robotInit unimplemented"); };
    public default void robotPeriodic() { log("robotPeriodic unimplemented"); };

    public default void driverStationConnected() { log("driverStationConnected unimplemented"); };

    public default void simulationInit() { log("simulationInit unimplemented"); };
    public default void simulationPeriodic() { log("simulationPeriodic unimplemented"); };

    public default void disabledInit() { log("disabledInit unimplemented"); };
    public default void disabledPeriodic() { log("disabledPeriodic unimplemented"); };
    public default void disabledExit() { log("disabledExit unimplemented"); };

    public default void autonomousInit() { log("autonomousInit unimplemented"); };
    public default void autonomousPeriodic() { log("autonomousPeriodic unimplemented"); };
    public default void autonomousExit() { log("autonomousExit unimplemented"); };

    public default void teleopInit() { log("teleopInit unimplemented"); };
    public default void teleopPeriodic() { log("teleopPeriodic unimplemented"); };
    public default void teleopExit() { log("teleopExit unimplemented"); };

    public default void testInit() { log("testInit unimplemented"); };
    public default void testPeriodic() { log("testPeriodic unimplemented"); };
    public default void testExit() { log("testExit unimplemented"); };
}
