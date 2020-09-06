//package org.firstinspires.ftc.teamcode.Tests.graph;
//
//import org.firstinspires.ftc.teamcode.Splines.Spline;
//import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;
//
//import java.util.ArrayList;
//
//public class Node {
//
//    // task stuff-----------------------------------------------------------------------------
//    public enum nodeType {SplineNoTheta, SplineTheta, ToPoint, ToPointK, RelativeMove, RobotAction}
//    public enum robotState {StoneClamped, StoneNotClamped, ArmHome}
//    public enum robotAction {GrabFoundation, ReleaseFoundation, DepositStone}
//
//    private nodeType type;
//    private SplineGenerator splineGenerator;
//    private Spline[] splines;   private double[] splineData;
//    private Spline thetaSpline; private double[] thetaSplineData;
//    private double theta, time; private double[] toPointData;
//    private robotAction action;
//
//    private boolean started = false;
//    private boolean wantsTimeReset = true;
//
//    // spline movement
//    public Node(double startx, double starty, double endx, double endy,
//                double starttheta, double endtheta, double startv, double endv,
//                double starta, double enda, double startw, double endw, double time, String name) {
//        type = nodeType.SplineNoTheta;
//        splineGenerator = new SplineGenerator();
//        splines = splineGenerator.SplineBetweenTwoPoints(startx, starty, endx, endy, starttheta, endtheta, startv, endv, starta, enda, startw, endw, time);
//        splineData = new double[] {startx, starty, endx, endy, starttheta, endtheta, startv, endv, starta, enda, startw, endw, time};
//        this.time = time; this.name = name;
//    }
//
//    public void addForcedTheta(double theta) {this.theta = theta;}
//
//    public void addThetaSpline(double startx, double startv, double starta, double endx, double endv, double enda) {
//        type = nodeType.SplineTheta;
//        thetaSpline = new Spline(startx, startv, starta, endx, endv, enda, time);
//        thetaSplineData = new double[] {startx, startv, starta, endx, endv, enda, time};
//    }
//
//    public void recalcSplines(double curx, double cury, double curtheta) {
//        splines = splineGenerator.SplineBetweenTwoPoints(curx, cury, splineData[2], splineData[3], curtheta, splineData[5], splineData[6], splineData[7], splineData[8], splineData[9], splineData[10], splineData[11], splineData[12]);
//        if (getType() == nodeType.SplineTheta) thetaSpline = new Spline(curtheta, thetaSplineData[2], thetaSplineData[3], thetaSplineData[4], thetaSplineData[5], thetaSplineData[6]);
//    }
//
//    // to point movement
//    public Node(double x, double y, double theta, String name) {
//        type = nodeType.ToPoint;
//        toPointData = new double[] {x, y, theta};
//        this.name = name;
//    }
//
//    // to point movement with forced k factor
//    public Node(double x, double y, double theta, double xk, double yk, double thetak, String name) {
//        type = nodeType.ToPointK;
//        toPointData = new double[] {x, y, theta, xk, yk, thetak};
//        this.name = name;
//    }
//
//    // movement relative to the robot
//    public Node(double x, double y, double theta, boolean isRelative, String name) {
//        type = nodeType.RelativeMove;
//        toPointData = new double[] {x, y, theta};
//        this.name = name;
//    }
//
//    // robot action
//    public Node(robotAction action, String name) {
//        type = nodeType.RobotAction;
//        this.action = action; this.name = name;
//    }
//
//    // do not reset time when task starts- use when running robot action while moving along spline
//    public void noTimeReset() {wantsTimeReset = false;}
//
//    // getter functions
//    public nodeType getType() {return type;}
//
//    public Spline[] getSplines() {return splines;}
//    public Spline getThetaSpline() {return thetaSpline;}
//
//    public double getTheta() {return theta;}
//
//    public double[] toPointData() {return toPointData;}
//
//    public robotAction getAction() {return action;}
//
//    public double getTime() {return time;}
//
//    public void setStarted() {started = true;}
//    public boolean isStarted() {return started;}
//    public boolean wantsTimeReset() {return wantsTimeReset;}
//
//    // graph stuff----------------------------------------------------------
//    private String name;
//    private int id;
//    private ArrayList<Edge> connectedEdges = new ArrayList<>();
//
//    public Node(String name) { /*test/dumb node*/ this.name = name;}
//
//    public void addEdge(Edge edge) {connectedEdges.add(edge);}
//
//    public ArrayList<Edge> getConnectedEdges() {return connectedEdges;}
//
//    public void setId(int id) {this.id = id;}
//
//    public int getId() {return id;}
//
//    @Override public String toString() {return name;}
//}
