package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallDetector {
    DigitalChannelImpl sensor;
    public enum BallPresence
    {
        PRESENT,
        ABSENT
    }
    private BallPresence presence;

    public BallDetector(HardwareMap hwMap)
    {
        sensor = hwMap.get(DigitalChannelImpl.class, "beam");
    }

    public boolean ballPresent(Telemetry telemetry)
    {
        return sensor.getState();
    }
}
