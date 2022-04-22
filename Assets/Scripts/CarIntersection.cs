using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class CarIntersection: MonoBehaviour
{
    public Graph graph;
    //TerrainManager terrain_manager;
    //public GameObject terrain_manager_game_object;

    List<Node> stoppingCorners = new List<Node>();
    private List<Node> stopLines = new List<Node>();

    public GameObject terrain_manager_game_object;
    //public Graph Graph
    //{
    //    get;      
    //}
    //public List<Node> StopLines
    //{
    //    get;
    //}

    void Start()
    {
        //terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        //graph = Graph.CreateGraph(terrain_manager.myInfo, terrain_manager.myInfo.x_N, terrain_manager.myInfo.z_N);
        
        if (graph == null)
        {
            Debug.Log("graph is null");

            graph = new Graph();
            graph = graph.CreateGraph();

            setImaginaryObstacles();
            stopNodes();
        }
    }
    public Graph getGraph() 
    { 
        return graph; 

    }
    public List<Node> getStopLines() { return stopLines;  }
    private void setImaginaryObstacles()
    {
            //Find corner obstacles 
            List<Node> cornerObstacles = new List<Node>();
            foreach(Node obstacle in graph.obstacleNodes){
                List<Node> neighbours;
                neighbours = graph.getNeighbours(obstacle);
                int nrWalkable = 0; 
                foreach (Node neighbour in neighbours){
                    if (neighbour.walkable){
                        nrWalkable++;
                    }
                }
                if (nrWalkable == 5) 
                {
                    Node possibleCorner1 = graph.nodes[obstacle.i+3, obstacle.j];
                    Node possibleCorner2 = graph.nodes[obstacle.i - 3, obstacle.j];
                    Node possibleCorner3 = graph.nodes[obstacle.i , obstacle.j + 3];
                    Node possibleCorner4 = graph.nodes[obstacle.i , obstacle.j - 3];

                    if (!(!possibleCorner1.walkable && !possibleCorner2.walkable && !possibleCorner3.walkable && !possibleCorner4.walkable))
                    {
                        cornerObstacles.Add(obstacle);
                    }
                    else
                    {
                        stoppingCorners.Add(obstacle);
                    }
                }
            }
            //Create imaginary obstacles 
            foreach(Node corner in cornerObstacles){
                
                List<Node> cornerNeighbours;
                cornerNeighbours = graph.getNeighbours(corner);
                foreach (Node neighbour in cornerNeighbours){
                    if (neighbour.i != corner.i)
                    {
                        int diff = neighbour.i - corner.i; // neighbour 7 and corner 8, diff = -1
                        if (!graph.nodes[corner.i - 2 * diff, corner.j].walkable)
                        {
                            if (graph.nodes[corner.i + 3 * diff, corner.j].walkable)
                            {
                                int newI = corner.i + diff; //corner 8 - (-1) = 9
                                Node newObstacle = graph.nodes[newI, corner.j];
                                newObstacle.walkable = false;
                                newObstacle = graph.nodes[newI + diff, corner.j];
                                newObstacle.walkable = false;
                            }
                        }
                    }
                    else
                    {
                        int diff = neighbour.j - corner.j;
                        if (!graph.nodes[corner.i, corner.j - 2 * diff].walkable)
                        {
                            if (graph.nodes[corner.i, corner.j + 3 * diff].walkable)
                            {
                                int newJ = corner.j + diff;
                                Node newObstacle = graph.nodes[corner.i, newJ];
                                newObstacle.walkable = false;
                                newObstacle = graph.nodes[corner.i, newJ + diff];
                                newObstacle.walkable = false;
                            }
                        }
                    }
                }
            }
        }
    private void stopNodes() //stoping nodes are saved in stopLines
    {
        foreach (Node corner in stoppingCorners)
        {
            for (int k = 1; k < 3; k++)
            {
                Node stopNode = graph.nodes[corner.i + k, corner.j];
                if (stopNode.walkable)
                    stopLines.Add(stopNode);
                stopNode = graph.nodes[corner.i, corner.j + k];
                if (stopNode.walkable)
                    stopLines.Add(stopNode);
            }
        }
    }

}   


