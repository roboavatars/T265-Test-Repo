//package org.firstinspires.ftc.teamcode.Tests.graph;
//
//import android.util.Log;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
//
//import java.util.ArrayList;
//
//public class Graph {
//
//    private ArrayList<Node> nodeList = new ArrayList<>();
//    private Edge[][] adjacencyMatrix;
//    private Robot robot;
//
//    private Node curNode;
//    private int counter = 0;
//    private ElapsedTime time;
//
//    public Graph(int nodes, Robot robot) {
//        adjacencyMatrix = new Edge[nodes][nodes];
//        time = new ElapsedTime();
//        this.robot = robot;
//    }
//
//    public void addNode(Node node) {
//        node.setId(nodeList.size());
//        nodeList.add(node);
//    }
//
//    public void addNode(Node... nodes) {
//        for (int i = 0; i < nodes.length; i++) {
//            Node node = nodes[i];
//            node.setId(nodeList.size());
//            nodeList.add(node);
//        }
//    }
//
//    public void addEdge(Edge edge) {
//        adjacencyMatrix[edge.getStart().getId()][edge.getEnd().getId()] = edge;
//    }
//
//    public void addEdge(Edge... edges) {
//        for (int i = 0; i < edges.length; i++) {
//            Edge edge = edges[i];
//            adjacencyMatrix[edge.getStart().getId()][edge.getEnd().getId()] = edge;
//        }
//    }
//
//    public void update() {
//
//        if (counter == 0) curNode = nodeList.get(0); // first task node must be added first, not really best idea though
//
//        // if task is not started, execute action based on task type
//        if (!curNode.isStarted()) {
//            log("Starting " + curNode + "("+curNode.getType()+")");
//            if (curNode.getType() == Node.nodeType.SplineNoTheta || curNode.getType() == Node.nodeType.SplineTheta) {
//                curNode.recalcSplines(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading);
//            } else if (curNode.getType() == Node.nodeType.ToPoint) {
//                robot.drivetrain.setTargetPoint(curNode.toPointData()[0], curNode.toPointData()[1], curNode.toPointData()[2]);
//            } else if (curNode.getType() == Node.nodeType.ToPointK) {
//                robot.drivetrain.setTargetPoint(curNode.toPointData()[0], curNode.toPointData()[1], curNode.toPointData()[2], curNode.toPointData()[3], curNode.toPointData()[4], curNode.toPointData()[5]);
//            } else if (curNode.getType() == Node.nodeType.RobotAction) {
//                Node.robotAction action = curNode.getAction();
//                log(action+"");
//                switch (action) {
//                    case DepositStone: robot.depositAuto();
//                    case GrabFoundation: robot.grabber.grabFoundation();
//                    case ReleaseFoundation: robot.grabber.releaseFoundation();
//                }
//            }
//            curNode.setStarted(); counter++;
//            if (curNode.wantsTimeReset()) time.reset();
//            else log("Time Not Reset");
//        }
//
//        // if task is started, execute action based on task type
//        if (curNode.isStarted()) {
//            if (curNode.getType() == Node.nodeType.SplineNoTheta || curNode.getType() == Node.nodeType.SplineTheta) {
//                double currentTime = Math.min(curNode.getTime(), time.seconds());
//                if (curNode.getType() == Node.nodeType.SplineNoTheta) robot.drivetrain.setTargetPoint(curNode.getSplines()[0].position(currentTime), curNode.getSplines()[1].position(currentTime), curNode.getTheta());
//                else robot.drivetrain.setTargetPoint(curNode.getSplines()[0].position(currentTime), curNode.getSplines()[1].position(currentTime), curNode.getThetaSpline().position(currentTime));
//            } else if (curNode.getType() == Node.nodeType.RelativeMove) {
//                robot.drivetrain.setTargetPoint(robot.drivetrain.x + curNode.toPointData()[0], robot.drivetrain.y + curNode.toPointData()[1], robot.drivetrain.currentheading + curNode.toPointData()[2]);
//            }
//        }
//
//        // check all edges connected to this node to see whether next action is ready
//        curNode = getNextNode();
//    }
//
//    public Node getNextNode() {
//        Node nextNode = curNode; // if no node in list ready, next node is current one
//
//        // for every connected edge
//        for (Edge edge : curNode.getConnectedEdges()) { // right now returns first ready node, later add a node priority system
//            boolean isReady = false;
//            ArrayList startConditions = edge.getStartConditions();
//
//            // for every start condition
//            for (int i = 0; i < startConditions.size(); i++) {
//                Object curCondition = startConditions.get(i);
//
//                // return appropriate value based on type of start condition, assumes all conditions are "OR" based
//                if (!isReady && edge.hasTimeStart() && curCondition instanceof Double) {
//                    isReady = time.seconds() > (double) curCondition;
//                    log("Time Condition Met");
//                }
//                if (!isReady && edge.hasRobotPosStart() && curCondition instanceof double[]) {
//                    double[] posCond = (double[]) curCondition;
//                    isReady = robot.drivetrain.isAtPoseAuto(posCond[0], posCond[1], posCond[2], posCond[3], posCond[4], posCond[5]);
//                    log("Position Condition Met");
//                }
//                if (!isReady && edge.hasStateStart() && curCondition instanceof Node.robotState) {
//                    Node.robotState endState = (Node.robotState) curCondition;
//                    isReady = endState == Node.robotState.StoneClamped && robot.stacker.stoneClamped
//                            || endState == Node.robotState.StoneNotClamped && !robot.stacker.stoneClamped
//                            || endState == Node.robotState.ArmHome && robot.stacker.isArmHome();
//                    log("State Condition Met");
//                }
//                if (!isReady && !edge.hasTimeStart() && !edge.hasRobotPosStart() && !edge.hasStateStart()) {
//                    isReady = true; log("Task Has No Start Condition");
//                }
//            }
//
//            // return next node if 1 start condition met
//            if (isReady) {
//                nextNode = edge.getEnd();
//                log("Following Edge " + edge);
//                break;
//            }
//        }
//        return nextNode;
//    }
//
//    public ArrayList<Node> getNodeList() {return nodeList;}
//
//    public void showRelationships() {
//        System.out.println(this);
//        for (Node node : getNodeList()) {
//            System.out.println(node + ": " + node.getConnectedEdges());
//        }
//    }
//    @Override public String toString() {
//        String s = "     ";
//        for (int i = 0; i < nodeList.size(); i++) {s += nodeList.get(i) + " ";}
//        s += "\n";
//        for (int i = 0; i < nodeList.size(); i++) {
//            s += nodeList.get(i) + ":";
//            for (int k = 0; k < 5 - (nodeList.get(i)+"").length(); k++) {s += " ";}
//            for (Edge j : adjacencyMatrix[i]) {s += (j!=null ? 1:0) + "   ";}
//            s += "\n";
//        }
//        return s;
//    }
//
//    private void log(String message) {Log.w("graph", message);}
//
//    // for debug
//    public static void main(String[] args) {
//        //NewAuto a = new NewAuto(); a.runOpMode();
//
//        /*Graph graph = new Graph(4);
//        Node n1 = new Node("1"); Node n2 = new Node("2");
//        Node n3 = new Node("3"); Node n4 = new Node("4");
//
//        Edge n1n2 = new Edge(n1, n2); Edge n2n3 = new Edge(n2, n3);
//        Edge n3n4 = new Edge(n3, n4); Edge n4n1 = new Edge(n4, n1);
//        Edge n1n3 = new Edge(n1, n3); Edge n2n4 = new Edge(n2, n4);
//        Edge n1n4 = new Edge(n1, n4); Edge n1n1 = new Edge(n1, n1);
//
//        graph.addNode(n1, n2, n3, n4);
//        graph.addEdge(n1n2, n2n3, n3n4, n4n1, n1n3, n2n4, n1n4, n1n1);
//
//        graph.showRelationships();*/
//    }
//    public Graph(int nodes) { /*test/dumb graph*/ adjacencyMatrix = new Edge[nodes][nodes];}
//}
