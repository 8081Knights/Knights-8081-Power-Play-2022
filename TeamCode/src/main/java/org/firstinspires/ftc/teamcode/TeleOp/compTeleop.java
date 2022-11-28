package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "Comp Tele")
public class compTeleop extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands = new RobotCommands();

    boolean aPress = false;

    enum ArmPos{
        HOME,
        BACK,
        OUT
    }

    enum ArmMove{
        H2O,
        H2B,
        O2H,
        B2H,
        O2B
    }

    enum slideHeight{
        Ground,
        Low,
        Middle,
        High,
        HighB
    }

    slideHeight slide = slideHeight.Ground;
    ArmPos pos = ArmPos.HOME;
    ArmPos prevPos = pos;

    int low = 500;
    int mid = 1000;
    int high = 1500;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.left_stick_x;



        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (-drive + strafe + twist),
                (-drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        robot.frontLeft().setPower(speeds[0]);
        robot.frontRight().setPower(speeds[1]);
        robot.backLeft().setPower(speeds[2]);
        robot.backRight().setPower(speeds[3]);

        

        if(gamepad1.dpad_right){
            slide = slideHeight.Ground;
        }
        if(gamepad2.dpad_right){
            slide = slideHeight.HighB;
        }
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            slide = slideHeight.Low;
        }
        if(gamepad1.dpad_left || gamepad2.dpad_left){
            slide = slideHeight.Middle;
        }
        if(gamepad1.dpad_up || gamepad2.dpad_up){
            slide = slideHeight.High;
        }




        switch(slide){
            case Ground:
                commands.slideChange(10);
                commands.armOut();

            case Low:
                commands.slideChange(low);
                commands.armOut();

            case Middle:
                commands.slideChange(mid);
                commands.armOut();

            case High:
                commands.slideChange(high);
                commands.armOut();

            case HighB:
                commands.slideChange(high);
                commands.armBack();

        }


    }
}
