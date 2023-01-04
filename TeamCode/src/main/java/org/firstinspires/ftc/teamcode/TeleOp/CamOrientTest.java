package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ConeOrientate;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
import org.firstinspires.ftc.teamcode.FusionFramework.GyroSensor;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Orientation Test")

public class CamOrientTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands = new RobotCommands();
    ConeOrientate detector = new ConeOrientate();
    OpenCvCamera camera;

    boolean aDown = false;
    boolean success = false;

    double pxDeg = 10;
    double coneDegree = 0;
    double coneTolerance = 4;
    double center = 395;


    @Override
    public void init() {
        robot.init(hardwareMap);
        camera = robot.getFrontWebCam();

        GyroSensor gyro = new GyroSensor(robot, false);

        gyro.initializeZeroPosition();



        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });



    }

    @Override
    public void loop() {
        commands.init(robot);

        if(gamepad1.a){
//            detector.sortCont();
//            telemetry.addData("Silly cone position: ", detector.conePos());
//            telemetry.update();
            aDown = true;
        }

//        Thread upPress = new Thread(() -> {
//            if(aDown && !gamepad1.a){
//                detector.sortCont();
//                success = true;
//                aDown = false;
//            }
//            else{
//                return;
//            }
//
//        });
//
//        upPress.start();


        if(aDown && !gamepad1.a){
            detector.sortCont(telemetry);
            success = true;
            aDown = false;

            coneDegree = (detector.conePos().x-center)/pxDeg;

        }


        if(success){
            telemetry.addLine("Yay ur stupid code worked");

        }

//        if(coneDegree>coneTolerance || coneDegree<-coneTolerance){
//            commands.turnGyro(coneDegree, 500);
//        }

        telemetry.addData("Silly cone angle: ", coneDegree);
        telemetry.addData("Silly cone position: ", detector.conePos());
        telemetry.update();




    }
}
