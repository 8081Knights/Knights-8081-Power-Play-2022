package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.RecordTests.controller.Controller;
import org.firstinspires.ftc.teamcode.RecordTests.monkeyc.MonkeyC;
import org.firstinspires.ftc.teamcode.RecordTests.monkeyc.MonkeyDo;

@TeleOp(name = "monkeyC")
public class monkeyTest extends OpMode {

    MonkeyC record = new MonkeyC();
    HardwareSoftware hMap = new HardwareSoftware();
    int t = 0;


    @Override
    public void init() {
        record.add(gamepad1, gamepad2);
        record.resumeTime();


    }

    @Override
    public void loop() {



    }
}
