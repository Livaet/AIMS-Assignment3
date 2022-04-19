﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
//using System.Linq;
//using System.Diagnostics;

/* TODO
- Move the cars to a line within a rectangular box and let them wait 
    - find the nodes for stopping
    - tell the cars where to stop
    - stop function
    - Try to park each car behind the previous car 
    - identify when a car is in front of you
- Let one section drive at a time in a formation 
- Wait until the intersection is empty before letting the next section drive. 
- Make a function always drive on the right
- Obstacle collision detection and avoidance (useful after the intersection)
*/

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        // Car Controller
        private CarController m_Car; // the car controller we want to use
        private Vector3 target_velocity;
        private Vector3 oldCarPosition;
        private Vector3 oldTargetPosition;

        // Terrain variables

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        // Friends and enemy variables
        public GameObject[] friends; // use these to avoid collisions

        public GameObject my_goal_object;

        //Graph and waypoint helpers
        public Graph graph;
        int nextWaypoint;



        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            graph = Graph.CreateGraph(terrain_manager.myInfo, terrain_manager.myInfo.x_N, terrain_manager.myInfo.z_N);

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = transform.position; // terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            friends = GameObject.FindGameObjectsWithTag("Car");

            List<Vector3> my_path = new List<Vector3>();
            nextWaypoint = 1;

            //my_path.Add(start_pos);

            //for (int i = 0; i < 3; i++)
            //{
            //    Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
            //    my_path.Add(waypoint);
            //}
            //my_path.Add(goal_pos);


            //// Plot your path to see if it makes sense
            //// Note that path can only be seen in "Scene" window, not "Game" window
            //Vector3 old_wp = start_pos;
            //foreach (var wp in my_path)
            //{
            //    //Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //    old_wp = wp;
            //}
            setImaginaryObstacles();
            //PathFinder.findPath(graph, start_pos, goal_pos, (360 - transform.eulerAngles.y + 90) % 360); // path is accessible through graph.path
        }


        private void FixedUpdate()
        {

            // // this is how you access information about the terrain
            // int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            // int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            // float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            // float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            // //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // Vector3 relVect = my_goal_object.transform.position - transform.position;
            // bool is_in_front = Vector3.Dot(transform.forward, relVect) > 0f;
            // bool is_to_right = Vector3.Dot(transform.right, relVect) > 0f;

            // if(is_in_front && is_to_right)
            //     m_Car.Move(1f, 1f, 0f, 0f);
            // if(is_in_front && !is_to_right)
            //     m_Car.Move(-1f, 1f, 0f, 0f);
            // if(!is_in_front && is_to_right)
            //     m_Car.Move(-1f, -1f, -1f, 0f);
            // if(!is_in_front && !is_to_right)
            //     m_Car.Move(1f, -1f, -1f, 0f);


            // this is how you control the car
            //m_Car.Move(1f, 1f, 1f, 0f);


            if (nextWaypoint < graph.path.Count)
            {
                drive();
            }

        }

        void OnDrawGizmos()
        {
            if (graph != null)
            {
                foreach (Node n in graph.nodes) // graph.path 
                {
                    Gizmos.color = (n.walkable) ? Color.blue : Color.red;
                    if (graph.path != null && graph.path.Contains(n))
                        Gizmos.color = Color.white;
                    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                }

                Node currentNode = graph.getNodeFromPoint(transform.position);
                //Debug.Log("CAR INITIAL NODE: [" + currentNode.i + "," + currentNode.j + "]");
                Gizmos.color = Color.cyan; // position of car
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));


            }

            if (graph.path != null)
            {
                for (int i = 0; i < graph.path.Count - 1; i++)
                {
                    Gizmos.color = Color.black;
                    Gizmos.DrawLine(graph.path[i].worldPosition, graph.path[i + 1].worldPosition);
                }
            }
        }


        void drive()
        {
            RaycastHit hit;
            Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, 50f);
            Debug.Log(hit.distance);

            float k_p = 0.3f;
            float k_d = 0.2f;
            // keep track of target position and velocity
            Vector3 target_position = graph.path[nextWaypoint].worldPosition;
            target_position[1] = 0.1f;
            Vector3 car_position = m_Car.transform.position;
            target_velocity = (target_position - oldTargetPosition) / Time.fixedDeltaTime;
            //target_velocity = (target_position - target_position) / Time.fixedDeltaTime;
            Vector3 car_velocity = (car_position - oldCarPosition) / Time.fixedDeltaTime;
            oldTargetPosition = target_position;
            oldCarPosition = car_position;

            // a PD-controller to get desired velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - car_velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);



            if (acceleration > 0)
            {
                m_Car.Move(steering, 1, 1, 0f);
            }
            else
            {
                m_Car.Move(steering, -1, -1, 0f);
            }


            Debug.DrawLine(car_position, target_position, Color.white);
            //Debug.DrawLine(transform.position, transform.position + car_velocity, Color.blue);
            //Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            //m_Car.Move(steering, acceleration, acceleration, 0f);

            if (Vector3.Distance(car_position, target_position) < 6)
            {
                nextWaypoint++;
            }
        }
        

        public void setImaginaryObstacles(){
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
                    Node possibleCorner1 = graph.nodes[obstacle.i+2, obstacle.j];
                    Node possibleCorner2 = graph.nodes[obstacle.i - 2, obstacle.j];
                    Node possibleCorner3 = graph.nodes[obstacle.i , obstacle.j + 2];
                    Node possibleCorner4 = graph.nodes[obstacle.i , obstacle.j - 2];

                    if (!(!possibleCorner1.walkable && !possibleCorner2.walkable && !possibleCorner3.walkable && !possibleCorner4.walkable))
                        cornerObstacles.Add(obstacle);
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

    }
}
