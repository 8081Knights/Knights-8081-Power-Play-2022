package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class controllerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Stick X:", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y:", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X:", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y:", gamepad1.right_stick_y);
            telemetry.update();
            sleep(500);
        }
    }
}
