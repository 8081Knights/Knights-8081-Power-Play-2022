package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@Disabled
@TeleOp(name = "Servo Tuning Test")
public class ServoTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper){
            robot.clawElbow().setPosition(1);
        }
        if(gamepad1.right_bumper){
            robot.clawElbow().setPosition(0);
        }

        if(gamepad1.a){
            robot.clawGrab().setPosition(0.6);
        }
        if(gamepad1.b){
            robot.clawGrab().setPosition(0);
        }
        if(gamepad1.x){
            robot.clawWrist().setPosition(0);
        }
        if(gamepad1.y){
            robot.clawWrist().setPosition(1);
        }

    }
}
