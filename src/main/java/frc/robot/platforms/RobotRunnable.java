package frc.robot.platforms;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.utilities.Util;

public interface RobotRunnable {
    private void log(String s) { Util.logf("(%s) %s\n", robotName(), s); }
    public default void logf(String pattern, Object... arguments) { Util.logf("(%s) " + pattern + "\n", robotName(), arguments); }
    public default void logfPeriodic(String pattern, Object... arguments) { if (Robot.count % 500 == 0) { logf(pattern, arguments); } }

    public default double getTriggerValue(CommandXboxController controller) { return Util.getSpeedFromTriggers(controller); }

    public default String robotName() { return "robotname-undefined"; }

    public default void robotInit() { logfPeriodic("robotInit unimplemented"); };
    public default void robotPeriodic() {};    

    public default void driverStationConnected() { logfPeriodic("driverStationConnected unimplemented"); };

    public default void simulationInit() { logfPeriodic("simulationInit unimplemented"); };
    public default void simulationPeriodic() { logfPeriodic("simulationPeriodic unimplemented"); };

    public default void disabledInit() { logfPeriodic("disabledInit unimplemented"); };
    public default void disabledPeriodic() { logfPeriodic("disabledPeriodic unimplemented"); };
    public default void disabledExit() { logfPeriodic("disabledExit unimplemented"); };

    public default void autonomousInit() { logfPeriodic("autonomousInit unimplemented"); };
    public default void autonomousPeriodic() { logfPeriodic("autonomousPeriodic unimplemented"); };
    public default void autonomousExit() { logfPeriodic("autonomousExit unimplemented"); };

    public default void teleopInit() { logfPeriodic("teleopInit unimplemented"); };
    public default void teleopPeriodic() { logfPeriodic("teleopPeriodic unimplemented"); };
    public default void teleopExit() { logfPeriodic("teleopExit unimplemented"); };

    public default void testInit() { logfPeriodic("testInit unimplemented"); };
    public default void testPeriodic() { logfPeriodic("testPeriodic unimplemented"); };
    public default void testExit() { logfPeriodic("testExit unimplemented"); };
}
