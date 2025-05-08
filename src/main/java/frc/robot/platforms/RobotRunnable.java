package frc.robot.platforms;

public interface RobotRunnable {
    public void robotInit();
    public void robotPeriodic();

    public void driverStationConnected();

    public void simulationInit();
    public void simulationPeriodic();

    public void disabledInit();
    public void disabledPeriodic();
    public void disabledExit();

    public void autonomousInit();
    public void autonomousPeriodic();
    public void autonomousExit();

    public void teleopInit();
    public void teleopPeriodic();
    public void teleopExit();

    public void testInit();
    public void testPeriodic();
    public void testExit();
    
}
