package org.firstinspires.ftc.teamcode.AutonomousPathGeneration;

import org.firstinspires.ftc.teamcode.util.PurePursuitPathPoint;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

/*
An A* Path finding algorithm that works within a cartesion grid system.
Necessary parameters:
 - Start Node
 - Goal Node
 - a full Map
 */
public class AStarAlgorithm {

    public Map map;

    public Node goalNode;
    public Node startNode;
    public Node currentNode;

    public ArrayList<Node> openNodes;   //set of nodes to be evaluated
    public ArrayList<Node> closedNodes; //set of nodes already evaluated

    public ArrayList<Node> finalPath;

    public AStarAlgorithm(Node startNode, Node goalNode, Map map) {
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.map = map;

        finalPath = new ArrayList<>();

        openNodes.add(startNode);
    }

    public ArrayList<Node> generatePath() {
        ArrayList<Node> neighborNodes = new ArrayList<>();

        while (openNodes.size() > 0) {
            //Find node with smallest fScore
            for (int i=0; i<openNodes.size(); i++) {
                if (openNodes.get(i).fScore() < currentNode.fScore() || (openNodes.get(i).fScore() == currentNode.fScore() && openNodes.get(i).hScore < currentNode.hScore)) {
                    currentNode = openNodes.get(i);
                }
            }

            if (currentNode == goalNode) {
                //Must reconstruct path by looking at each node's parentNode and return the path
                retraceOptimalPath();
                return finalPath;
            }

            openNodes.remove(currentNode);
            closedNodes.add(currentNode);

            neighborNodes = map.getNeighbors(currentNode);
            for (Node neighborNode : neighborNodes) {
                if (neighborNode != startNode && neighborNode.usage != NodeUsage.OBSTACLE && neighborNode.usage != NodeUsage.SAFETY && neighborNode.status != NodeStatus.CLOSED) {  //if the node is open and traversable
                    //Calculating shortest path from start node to neighbor node
                    double updatedMovementCostToNeighbor = currentNode.gScore + getDistance(currentNode, neighborNode);
                    //if the neighbor is already an open node, or if there is an updated, shorter path from the startNode to the neightbor node:
                    if (!openNodes.contains(neighborNode) || updatedMovementCostToNeighbor < neighborNode.gScore) {
                        //Updating the fScore of the neighbor node with the shortest possible path
                        neighborNode.gScore = updatedMovementCostToNeighbor;
                        neighborNode.hScore = getDistance(neighborNode, startNode);
                        //Set the parentNode of the neighborNode to the currentNode; this is so when we retrace the optimal path, we get the actual optimal path
                        neighborNode.parentNode = currentNode;

                        if (!openNodes.contains(neighborNode) ) {
                            openNodes.add(neighborNode);
                        }
                    }
                }
            }
        }
        return null;    //There is no possible path
    }

    public double getDistance(Node nodeA, Node nodeB) {
        //Find the difference between the 2 nodes in the x and y axes
        double yDiff = Math.abs(nodeB.col - nodeA.col);
        double xDiff = Math.abs(nodeB.row - nodeA.row);

        //The number of diagonal movements required is equal to the smaller of the 2 differences
        int numDiagonals = Math.min((int) yDiff, (int) xDiff);
        //the number of straights is equal to the larger og the 2 differences - the number of diagonal movements
        int numStraights = Math.max((int) yDiff, (int) xDiff) - numDiagonals;

        double distance = numDiagonals * 1.414 + numStraights;
        return distance;
    }

    public void retraceOptimalPath() {
        Node currentNode = goalNode;
        while (currentNode != startNode) {
            finalPath.add(currentNode);
            currentNode = currentNode.parentNode;
        }
        Collections.reverse(finalPath);
    }

}
