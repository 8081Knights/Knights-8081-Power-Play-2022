package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class MecanumTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor BackLeft = hardwareMap.dcMotor.get("BackLeft");
        DcMotor FrontRight = hardwareMap.dcMotor.get("FrontRight");
        DcMotor BackRight = hardwareMap.dcMotor.get("BackRight");


        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FrontLeft.setPower(frontLeftPower);
            BackLeft.setPower(backLeftPower);
            FrontRight.setPower(frontRightPower);
            BackRight.setPower(backRightPower);
        }
    }
}