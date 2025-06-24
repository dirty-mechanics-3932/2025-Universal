package frc.robot.platforms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorSparkMax;

public class BlondeMini implements RobotRunnable {
  private final MotorSparkMax m_motorSparkMax;
  private final CommandXboxController m_controller;

  public BlondeMini(CommandXboxController controller) {
    m_motorSparkMax = new MotorSparkMax("TestMax", 20, -1, controller, false, false);
    new DrivetrainSRX(controller.getHID());
    this.m_controller = controller;
  }

  @Override
  public String robotName() {
    return "BlondeMini";
  }

  @Override
  public void robotInit() {
    Command blondeMove = Commands.run(() -> m_motorSparkMax.setSpeed(getTriggerValue(m_controller)), m_motorSparkMax);
    blondeMove.ignoringDisable(true).schedule();
  }

  @Override
  public void testInit() {
    m_motorSparkMax.setLogging(true);
    m_motorSparkMax.setTestMode(true);
  }
}
