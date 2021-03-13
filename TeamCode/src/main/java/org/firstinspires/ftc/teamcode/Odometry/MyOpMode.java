package org.firstinspires.ftc.teamcode.Odometry;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Odometry.RobotMovement.followCurve;

public class MyOpMode extends  OpMode{

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,0,1.0,1.0,50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180,180,1.0,1.0,50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(220,180,1.0,1.0,50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(280,50,1.0,1.0,50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180,0,1.0,1.0,50, Math.toRadians(50), 1.0));
        //followCurve(allPoints, Math.toRadians(90));
        //At the end, pure pursuit fails because it is based on looking ahead and you have nowhere to go at the end
        //If you extend the line, then you go to target point and point to extended path
        //Once you get close to target point, set follow point to last point on list
    }
}
