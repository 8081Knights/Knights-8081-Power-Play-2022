package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="Encoder Motor Test")
public class EncoderTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Front Right: ", robot.frontRight().getCurrentPosition());
        telemetry.addData("Front Left: ", robot.frontLeft().getCurrentPosition());
        telemetry.addData("Back Right: ", robot.backRight().getCurrentPosition());
        telemetry.addData("Back Left: ", robot.backLeft().getCurrentPosition());

        telemetry.update();

    }
}