package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static frc.robot.utilities.Util.logf;

public class CanHealthChecker {

    private final Map<Integer, SparkMax> sparkDevices = new HashMap<>();
    private final Map<Integer, TalonFX> talonFXDevices = new HashMap<>();

    private final Map<Integer, String> sparkNames = new HashMap<>();
    private final Map<Integer, String> talonFXNames = new HashMap<>();

    
  

    // Add devices manually
    public void registerSparkMax(int canID, String name) {
        sparkDevices.put(canID, new SparkMax(canID, MotorType.kBrushless));
        sparkNames.put(canID, name);
    }

    public void registerTalonFX(int canID, String name) {
        talonFXDevices.put(canID, new TalonFX(canID));
        talonFXNames.put(canID, name);
    }

    // Run the check
    public void runCheck() {
        logf("\n");
        logf("***************************************\n");
        logf("[CAN HEALTH CHECK]\n");
        checkSparks();
        checkTalons();
        logf("*****************************************\n\n");
        
    }

    private void checkSparks() {
        for (Map.Entry<Integer, SparkMax> entry : sparkDevices.entrySet()) {
            int id = entry.getKey();
            SparkMax device = entry.getValue();
            String name = sparkNames.get(id);
            logf("Name:%s\n", name);
            if (device.getFirmwareVersion() == 0) {
                logf("SPARK MAX ID:%d name:%s NOT RESPONDING\n", id , name);
            } else {
                logf("SPARK MAX ID:%d name:%s OK\n", id, name);
            }
        }
    }

    private void checkTalons() {
        for (Map.Entry<Integer, TalonFX> entry : talonFXDevices.entrySet()) {
            int id = entry.getKey();
            TalonFX device = entry.getValue();
            String name = sparkNames.get(id);
            if (device.getDeviceID() == 0) {
                logf("TalonFX ID:%d name:%s NOT RESPONDING\n");
            } else {
               logf("TalonFX ID:%d name:%s OK\n", id, name);
            }
        }
    }
}
