package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;

public class LedSubsystem extends SubsystemBase {
    private boolean changed = false;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int numberOfLeds = Config.numberOfLeds;

    public enum STATE {
        SET_IDLE_COLOR, IDLE, FLASH, SET_HOLDING_COLOR, HOLDING, SET_INDEXED_COLOR,
        INDEXED, SET_ERROR_COLOR, ERROR
    };

    STATE state = STATE.IDLE;

    boolean lastNoteState = false;
    boolean light = false;
    boolean on = true;

    public LedSubsystem() {
        initNeoPixel();
        setAllColors(0, 0, 0);
    }

    @Override
    public void periodic() {
        if (changed) {
            led.setData(ledBuffer);
        }
        changed = false;
    }

    private void initNeoPixel() {
        logf("Init Led subsystem number:%d port:%d\n", numberOfLeds, Robot.config.PWMLedStrip);
        led = new AddressableLED(Robot.config.PWMLedStrip);
        ledBuffer = new AddressableLEDBuffer(numberOfLeds);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    private void setLed(int num, int r, int g, int b) {
        if (num < numberOfLeds && num >= 0) {
            try {
                ledBuffer.setRGB(num, r, g, b);
            } catch (Exception e) {
                logf("***** led number %d out of range\n", num);
            }
        } else {
            logf("!!!!!! led number %d out of range\n", num);
        }
    }

    public void setMode(STATE state) {
        this.state = state;
    }

    public void setTestCaseColors(int mode, int r, int g, int b) {
        setRangeOfColor(mode, 1, r, g, b);
    }

    public void setAllColors(int r, int g, int b) {
        for (int i = 0; i < numberOfLeds; i++) {
            setLed(i, r, g, b);
        }
        changed = true;
    }

    public void setRangeOfColor(int start, int num, int r, int g, int b) {
        for (int i = start; i <= start + num; i++) {
            setLed(i, r, g, b);
        }
        changed = true;
    }

    public void setRangeOfColor(int start, int num, int total, int r, int g, int b) {
        for (int i = start; i < start + total; i++) {
            if (i < num + start)
                setLed(i, r, g, b);
            else
                setLed(i, 0, 0, 0);
        }
        changed = true;
    }

    public void setOneLed(int num, int r, int g, int b) {
        setLed(num, r, g, b);
        changed = true;
    }
}
