//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.HardwareSoftware;
//
//public class odometryDistanceCheck {
//
//    HardwareSoftware robot = new HardwareSoftware();
//
//    public int odoDistanceForward() {
//
////        int distanceTravelled = (int)(robot.encoder().getCurrentPosition() / 522);
//
//
//        return distanceTravelled;
//
//    }
//
//    public int odoDistanceTurned() {
//
//        int distanceTravelledCm = (int)(robot.turnEncoder().getCurrentPosition() / 522);
//
//        int radius = 0;
//
//        double circumference = 2*radius*Math.PI;
//
//        int degreeCm = (int)(circumference / 360);
//
//        int degreesTurned = (int)(distanceTravelledCm / degreeCm);
//
//
//        return degreesTurned;
//
//    }
//}
