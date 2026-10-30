package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BioBuzzauto", group = "Autonomous")
public class BioBuzzauto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose position1 = poseFactory.of(57.6134, 8.4535, 90);
    private final Pose point1 = poseFactory.of(57.3877, 27.4443, 90.6808);
    private final Pose point2 = poseFactory.of(20.2973, 27.2486, -179.6977);
    private final Pose point3 = poseFactory.of(20.3388, 46.7225, -90.1221);
    private final Pose point4 = poseFactory.of(57.2512, 46.7565, -179.9473);
    private final Pose point5 = poseFactory.of(57.562, 8.1724, -89.5385);

    private final Pose point6 = poseFactory.of(57.6134, 8., 90);

    private final Pose point7= poseFactory.of (57.6134, 44, 90);
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower,Paths.line(point6, point7).tangent())
//                follow(follower, path1()),
//                follow(follower, path2()),
//                follow(follower, path3()),
//                follow(follower, path4()),
//                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(position1);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(position1, point1).tangent();
    }

    public Path path2() {
        return Paths.line(point1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }
}
