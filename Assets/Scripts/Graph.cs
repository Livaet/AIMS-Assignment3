using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Graph
{
    public int i_size;
    public int j_size;
    public float x_low;
    public float x_high;
    public float z_low;
    public float z_high;
    public float x_unit;
    public float z_unit;
    public Node start_node;
    public Node goal_node;
    public List<Node> path;
    public List<Node> obstacleNodes = new List<Node>();
    int numberOfCars = 50;


    public int[,] graphTraversabilityMatrix;

    public HashSet<GraphEdge> edges;
    //public Node[,] nodes;
    public Node[,] nodes; 
    public Node[][,] allCarNodes; // first dimension is the carId, second dimension i and third j. 
    public Node[,] newNodes;


    public static Graph instance;

    
    public Graph(){
    }
    public Graph(int i_size, int j_size, float x_low, float x_high, float z_low, float z_high)
    {
        this.i_size = i_size;
        this.j_size = j_size;
        this.x_low = x_low;
        this.x_high = x_high;
        this.z_low = z_low;
        this.z_high = z_high;
        this.x_unit = (x_high - x_low) / i_size;
        this.z_unit = (z_high - z_low) / j_size;
        this.edges = new HashSet<GraphEdge>();
        this.nodes = new Node[i_size, j_size];
        this.graphTraversabilityMatrix = new int[i_size, j_size];
        this.allCarNodes = new Node[numberOfCars][,];
        this.newNodes = new Node[i_size, j_size];


    }

    public void createObstacleList()
    {
        foreach (Node node in nodes)
        {
            if (!node.walkable)
            {
                obstacleNodes.Add(node);
            }
 
        }
    }
    public void AddEdge(GraphEdge edge)
    {
        edges.Add(edge);
    }

    public bool AreConnected(Node node1, Node node2)
    {
        foreach (GraphEdge edge in edges)
        {
            if (edge.start_node.Equals(node1) && edge.end_node.Equals(node2))
            {
                return true;
            }
            if (edge.start_node.Equals(node2) && edge.end_node.Equals(node1))
            {
                return true;
            }
        }
        return false;
    }

    public void printTraversability()
    {
        string traversability_string = "";
        for (int i = 0; i < i_size; i++)
        {
            for (int j = 0; j < j_size; j++)
            {
                traversability_string += (graphTraversabilityMatrix[i, j] + " ");
            }
            traversability_string += "\n";
        }
        Debug.Log(traversability_string);
    }
    //public void upgradeGraph()
    //{
        
    //    List<Node> usedNodes = new List<Node>();
    //    for (int i = 0; i < nodes.GetLength(0); i++)
    //    {
    //        for (int j = 0; j < nodes.GetLength(1); j++)
    //        {
    //            Node node = nodes[i, j];
    //            int rightNode = i + 1;
    //            int bottomNode = j + 1;
    //            if (usedNodes.Contains(node))
    //            {
    //                continue;
    //            }
    //            if(rightNode >= nodes.GetLength(0) && bottomNode >= nodes.GetLength(1))
    //            {
    //                continue;
    //            }
    //            if (nodes[i, j].walkable && nodes[rightNode, j].walkable && nodes[i, bottomNode].walkable && nodes[rightNode, bottomNode].walkable)
    //            {
    //                usedNodes.Add(node);
    //                usedNodes.Add(nodes[rightNode, node.j]);
    //                usedNodes.Add(nodes[node.i, bottomNode]);
    //                usedNodes.Add(nodes[rightNode, bottomNode]);
    //                nodes[i, j].x_pos = (node.x_pos + nodes[node.i + 1, node.j].x_pos) / 2;
    //                nodes[i, j].z_pos = (node.z_pos + nodes[node.i, node.j + 1].z_pos) / 2;
    //                nodes[i+1, j] = nodes[i, j];
    //                nodes[i, j+1] = nodes[i, j];
    //                nodes[i+1, j+1] = nodes[i, j];
    //            }
    //            else
    //            {
    //                usedNodes.Add(nodes[i, j]);
    //            }                
    //        }
    //    }
    //}
    public Graph CreateGraph() //TerrainInfo terrainInfo, int x_N, int z_N
    {


        // x_N, z_N are the number of boxes in the graph in the respective directions 
        // x_dim, z_dim are the dimensions of one box in the graph
        //if (terrainInfo == null)
        //{
        //Debug.Log("terrain_manager is null");
        //    return null;
        //}
        Debug.Log("tM" + TerrainManager.instance);
        TerrainInfo terrainInfo = TerrainManager.instance.myInfo;
        Debug.Log("terrain Info: " + terrainInfo);
        int x_N = terrainInfo.x_N * 2;
        int z_N = terrainInfo.z_N * 2;
        //Debug.Log("myFunction");
        float x_len = terrainInfo.x_high - terrainInfo.x_low;
        float z_len = terrainInfo.z_high - terrainInfo.z_low;
        float x_dim = x_len / x_N;
        float z_dim = z_len / z_N;
        
        Debug.Log("x_N: " + x_N);
        Debug.Log("z_N: " + z_N);

        Graph graph = new Graph(x_N, z_N, terrainInfo.x_low, terrainInfo.x_high, terrainInfo.z_low, terrainInfo.z_high);
        //in this graph, each "box" is actually 4 quadrants 
       
        //graph.start_node = new Node((int)(terrainInfo.start_pos[0] / x_num), (int)(terrainInfo.start_pos[2] / z_num), terrainInfo.start_pos[0], terrainInfo.start_pos[2]);
        //graph.goal_node = new Node((int)(terrainInfo.goal_pos[0] / x_num), (int)(terrainInfo.goal_pos[2] / z_num), terrainInfo.goal_pos[0], terrainInfo.goal_pos[2]);

        for (int i = 0; i < x_N; i++)
        {

            for (int j = 0; j < z_N; j++)
            {
                float x_center = terrainInfo.x_low + x_dim * (i + 0.5f);
                float z_center = terrainInfo.z_low + z_dim * (j + 0.5f);
                Node node = new Node(i, j, x_center, z_center);
                graph.nodes[i, j] = node;
                int i_index = terrainInfo.get_i_index(x_center);
                int j_index = terrainInfo.get_j_index(z_center);
                Collider[] collision = Physics.OverlapSphere(new Vector3(x_center, 1, z_center), Math.Max(x_dim/2, z_dim/2));
                bool walkable = true;
                foreach (Collider c in collision)
                {
                    if (c.name == "Cube")
                    {
                        walkable = false;
                        break;
                    }
                }
                walkable = true; //TODO understand why if I implement this (hence, if i remove this line) no path is found on terrain B
                if (terrainInfo.traversability[i_index, j_index] == 0 && walkable)
                {
                    //Gizmos.color = Color.blue;
                    graph.graphTraversabilityMatrix[i, j] = 0; //0 means that the node is traversable
                    if (i == x_N || j == z_N)
                        continue;
                    int next_i_index = terrainInfo.get_i_index(x_center + x_dim); //right
                    if (terrainInfo.traversability[next_i_index, j_index] == 0)
                    {
                        Node right_node = new Node(i + 1, j, x_center + x_dim, z_center);
                        GraphEdge edge = new GraphEdge(node, right_node);
                        GraphEdge edge_reverse = new GraphEdge(right_node, node);
                        graph.AddEdge(edge);
                        graph.AddEdge(edge_reverse);
                        graph.graphTraversabilityMatrix[i + 1, j] = 0;
                        //Debug.Log("RIGHT:" + "Adding edge from node (" + node.i + "," + node.j + ") to node (" + right_node.i + "," + right_node.j + ")");
                    }
                    else
                    {
                        //Debug.Log("RIGHT:" + "Not adding edge from node (" + i + "," + j + ") to node (" + (i + 1) + "," + j + ")");
                        graph.graphTraversabilityMatrix[i + 1, j] = 1;
                    }

                    int next_j_index = terrainInfo.get_j_index(z_center + z_dim); //up
                    if (terrainInfo.traversability[i_index, next_j_index] == 0)
                    {
                        Node up_node = new Node(i, j + 1, x_center, z_center + z_dim);
                        GraphEdge edge = new GraphEdge(node, up_node);
                        GraphEdge edge_reverse = new GraphEdge(up_node, node);
                        graph.AddEdge(edge);
                        graph.AddEdge(edge_reverse);
                        graph.graphTraversabilityMatrix[i, j + 1] = 0;
                        //Debug.Log("UP: Adding edge from node (" + node.i + "," + node.j + ") to node (" + up_node.i + "," + up_node.j + ")");

                    }
                    else
                    {
                        //Gizmos.color = Color.red;
                        //Debug.Log("RIGHT:" + "Not adding edge from node (" + i + "," + j + ") to node (" + (i) + "," + (j + 1) + ")");
                        graph.graphTraversabilityMatrix[i, j + 1] = 1;

                    }
                    //Gizmos.DrawCube(new Vector3(x_center, 0, z_center), new Vector3(x_len - 0.1f, 0.1f, z_len - 0.1f));



                    /*int next_i_index = terrain_manager.get_i_index(x_center - x_unit); //left
                    if (terrain_manager.traversability[next_i_index, j_index] == 0){
                        Node left_node = new Node(i-1, j, x_center - x_unit, z_center);
                        GraphEdge edge = new GraphEdge(node, left_node);
                    }
                    int next_j_index = terrain_manager.get_j_index(z_center + z_unit); //down
                    if (terrain_manager.traversability[i_index, next_j_index] == 0){
                        Node down_node = new Node(i, j-1, x_center, z_center - z_unit);
                        GraphEdge edge = new GraphEdge(node, down_node);
                    }
                    */

                }
                else
                {
                    node.walkable = false;
                    graph.graphTraversabilityMatrix[i, j] = 1; //1 means that the node is not traversable
                }

            }
        }
        //Here we would like to put the goal sphere to unwalkable 
        /*Collider [] goal_spheres = terrain_manager_game_object.GetComponents<Collider>(); //change this to get all spheres 
        foreach (Collider c in goal_spheres)
        {
            if (c.name == "Sphere")
            {
                Node goal_node = getNodeFromPoint(c.transform.position);
                goal_node.walkable = false;
                Debug.Log("Found a sphere");
            }
        }*/

        foreach(Node node in graph.nodes)
        {
            node.neighbours = graph.getNeighbours(node);
        }
        graph.createObstacleList();

        

        for (int k=0; k < numberOfCars; k++)
        {

            foreach(Node n in graph.nodes)
            {
                graph.newNodes[n.i, n.j] = n.clone();
                
            }

//            Array.Copy(graph.nodes, graph.newNodes, graph.nodes.Length);
            graph.allCarNodes[k] = graph.newNodes;
        }
        
        return graph;
    }

    public Node getNodeFromPoint(Vector3 position)
    {
        //Debug.Log("X: " + position.x);
        //Debug.Log(x_low);
        //Debug.Log(x_unit);
        //Debug.Log("Z: " + position.z);
        //Debug.Log(z_low);
        //Debug.Log(z_unit);

        int i = (int)((position.x - x_low) / x_unit);
        int j = (int)((position.z - z_low)/ z_unit);
        //Debug.Log(i);
        //Debug.Log(j);
        //Debug.Log("Size i: " + nodes.GetLength(0) + " Size j: " + nodes.GetLength(1));
        return nodes[i, j];
    }

    public List<Node> getNeighbours(Node node, bool print=false)
    {
        List<Node> toReturn = new List<Node>();
        
            for (int i = -1; i < 2; i++)
            {
                int current_i = node.i + i;
                if (current_i < 0 || current_i >= nodes.GetLength(0))
                    continue;
                for (int j = -1; j < 2; j++)
                {
                    int current_j = node.j + j;
                    if (current_j < 0 || current_j >= nodes.GetLength(1) || i == 0 && j == 0)
                        continue;
                    if (true) //i == 0 || j == 0
                    {
                        toReturn.Add(nodes[current_i, current_j]);
                    }

                    if (!nodes[current_i, current_j].walkable)
                    {
                        if (node.i == current_i)
                        {
                            node.wallClosenessCost += 12 * x_unit + 0.01f; //it means that the wall is above or behind, better go in the other direction
                        }
                        else if (node.j == current_j)
                            node.wallClosenessCost += 12 * z_unit + 0.01f; //it means that the wall is next or before, better go in the other direction
                        else
                            node.wallClosenessCost += 2 * (float)Math.Sqrt(x_unit * x_unit + z_unit * z_unit); //it means the wall is diagonally placed, better go in the other direction
                        if (print)
                            Debug.Log("Current node penalty: " + node.wallClosenessCost);
                    }
                }
            }
        
        return toReturn;
    }
    
}