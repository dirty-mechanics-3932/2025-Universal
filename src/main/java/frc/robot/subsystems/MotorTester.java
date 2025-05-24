package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorTester {
    public enum Motors {
        FLEX, MAX, KRAKEN, SRX;

        public Motors next() {
            Motors[] values = Motors.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }
    private Motors m_selectedMotor = Motors.FLEX; // Set default motor for testing

    private final MotorFlex m_flex;
    private final MotorSparkMax m_spark;
    private final MotorKraken m_kraken;
    private final MotorSRX m_srx;

    public MotorTester(MotorFlex flex, MotorSparkMax spark, MotorKraken kraken, MotorSRX srx) {
        m_flex = flex;
        m_spark = spark;
        m_kraken = kraken;
        m_srx = srx;
    }

    private void _resetAllMotorState() {
        m_flex.setTestMode(false);
        m_flex.setLogging(false);
        m_flex.setSmartTicks(0);
        m_spark.setTestMode(false);
        m_spark.setLogging(false);
        m_spark.setSmartTicks(0);
        m_kraken.setTestMode(false);
        m_kraken.setLogging(false);
        m_kraken.setSmartTicks(0);
        m_srx.setTestMode(false);
        m_srx.setLogging(false);
        m_srx.setSmartTicks(0);
    }

    public MotorTester.Motors selectNextMotor() throws Exception {
        _resetAllMotorState();

        m_selectedMotor = m_selectedMotor.next();
        switch (m_selectedMotor) {
        case FLEX:
            m_flex.setTestMode(true);
            m_flex.setLogging(true);
            m_flex.setSmartTicks(2);
            break;
        case MAX:
            m_spark.setTestMode(true);
            m_spark.setLogging(true);
            m_spark.setSmartTicks(2);
            break;
        case KRAKEN:
            m_kraken.setTestMode(true);
            m_kraken.setLogging(true);
            m_kraken.setSmartTicks(1);
            break;
        case SRX:
            m_srx.setTestMode(true);
            m_srx.setLogging(true);
            m_srx.setSmartTicks(2);
            break;
        default:
            throw new Exception("unexpected MotorTester.Motors type");
        }

        SmartDashboard.putString("MotorTester_Selected", m_selectedMotor.toString());

        return m_selectedMotor;
    }

}
