package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BallDetector;
import org.firstinspires.ftc.teamcode.subsystems.TestBenchColor;

@TeleOp
public class BeamBreakTest extends OpMode {
    BallDetector bench;

    @Override
    public void init()
    {
        bench = new BallDetector(hardwareMap);
    }

    @Override
    public void loop()
    {
        boolean state = bench.update();
        telemetry.addData("ball detected: ", state);
    }
}