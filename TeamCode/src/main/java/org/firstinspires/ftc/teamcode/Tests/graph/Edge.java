//package org.firstinspires.ftc.teamcode.Tests.graph;
//
//import java.util.ArrayList;
//
//public class Edge {
//
//    // start conditions stuff-------------------------------------------------------
//    private ArrayList startConditions = new ArrayList();
//    private boolean hasTimeStart = false;
//    private boolean hasRobotPosStart = false;
//    private boolean hasStateStart = false;
//
//    public void setTimeStartCondition(double time) {
//        hasTimeStart = true; startConditions.add(time);
//    }
//
//    public void setPosStartCondition(double x, double y, double theta, double xtol, double ytol, double thetatol) {
//        hasRobotPosStart = true; startConditions.add(new double[]{x, y, theta, xtol, ytol, thetatol});
//    }
//
//    public void setStateStartCondition(Node.robotState robotState) {
//        hasStateStart = true; startConditions.add(robotState);
//    }
//
//    public ArrayList getStartConditions() {return startConditions;}
//    public boolean hasTimeStart() {return hasTimeStart;}
//    public boolean hasRobotPosStart() {return hasRobotPosStart;}
//    public boolean hasStateStart() {return hasStateStart;}
//
//    // graph stuff-----------------------------------------------------
//    private Node start, end;
//
//    public Edge(Node start, Node end) {
//        this.start = start; this.end = end;
//        start.addEdge(this);
//    }
//
//    public Node getStart() {return start;}
//
//    public Node getEnd() {return end;}
//
//    @Override public String toString() {return start + "-" + end;}
//}
