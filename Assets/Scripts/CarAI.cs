using System.Collections;
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
- Make fake obstacles so the cars don't crash into the line of cars waiting //this is done below
*/

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        // Car Controller
        private CarController m_Car; // the car controller we want to use
        Vector3 carSize = new Vector3(2.43f, 0.41f, 4.47f);
        int carId;

        // Game controller
        public GameObject currentGameObject;

        // Terrain variables
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        TerrainInfo terrainInfo;


        // pathing variables
        private CarIntersection intersection;
        private Graph graph;
        public List<Node> stopLines;
        private List<Node> path;
        private List<Node> preliminaryPath;



        // Friends and enemy variables
        public GameObject[] friends; // use these to avoid collisions
        public GameObject my_goal_object;

        //Driving helpers
        int nextWaypoint;
        private Vector3 target_velocity;
        private Vector3 oldCarPosition;
        private Vector3 oldTargetPosition;
        public bool carInFront;
        float acceleration;
        float footbrake;
        float handbrake = 0f;
        float steering;

        //debug helpers
        bool debugOn = true;
        int debugThisCarId = 20;


        //Obstacle avoidance helpers
        bool had_hit_backward = false;
        bool had_hit = false; // They haven't hit anything. 
        int recovery = 0;


        //Status keepers
        bool stopAndWait = false;
        int layerMask;

        //static bool testing = true;

        Rigidbody body;



        void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            //graph = Graph.CreateGraph(terrain_manager.myInfo, terrain_manager.myInfo.x_N, terrain_manager.myInfo.z_N);  // moved to car intersection
            GameObject intersectionObject = GameObject.FindGameObjectsWithTag("Intersection")[0];
            intersection = intersectionObject.GetComponent<CarIntersection>(); //Maybe not get component 
            graph = intersection.getGraph();
            stopLines = intersection.getStopLines();

            Vector3 start_pos = transform.position; // terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = my_goal_object.transform.position;
            intersection.setSphereToObstacle(goal_pos);

            friends = GameObject.FindGameObjectsWithTag("Car");

            nextWaypoint = 1;
            //layerMask = LayerMask.GetMask("Cars");
            layerMask = 1 << 6;
            body = GetComponent<Rigidbody>();
            terrainInfo = TerrainManager.instance.myInfo;

            setCarObject();
            setCarId();

            //setImaginaryObstacles(); // moved to car intersection
            //stopNodes(); // moved to car intersection
            //for (int i = 19; i <21; i++)
            //{
            //    if (currentGameObject == friends[i])
            //    {
            //        preliminaryPath = PathFinder.findPath(graph,carId, start_pos, goal_pos, (360 - transform.eulerAngles.y + 90) % 360);
            //        Debug.Log("starting node is : " + graph.getNodeFromPoint(start_pos).i + " " + graph.getNodeFromPoint(start_pos).j);
            //        Debug.Log("final destination is: " + goal_pos);
            //        Debug.DrawLine(start_pos, goal_pos);
            //    }
            //}

            preliminaryPath = PathFinder.findPath(graph, carId, start_pos, goal_pos, (360 - transform.eulerAngles.y + 90) % 360);
            path = pathAdjustRightWall(preliminaryPath);

            graph.allCarNodes[0][0, 0].walkable = true;
            

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

            //if (path != null) {
            //    for (int i = 0; i < path.Count - 1; i++)
            //    {
            //        Debug.DrawLine(path[i].worldPosition, path[i + 1].worldPosition, Color.cyan);
            //    }
            //}
            
            //CarInFront();
            carInFront = false;
            if (!carInFront)
            {
                if (nextWaypoint < path.Count)
                {
                    if (stopLines.Contains(graph.getNodeFromPoint(path[nextWaypoint + 1].worldPosition)))
                    {
                        stopAndWait = true;

                    }
                    if (stopAndWait)
                    {
                        stop();
                    }
                    else
                    {
                        drive();
                    }
                }
            }
            else
            { stop(); }
            
        }



        public void setCarObject()
        {
            foreach (GameObject car in friends)
            {
                if (car.transform.position == m_Car.transform.position)
                {
                    currentGameObject = car;
                    break;
                }
            }
        }

       
        void OnDrawGizmos()
        {
            if (graph != null)
            {
                //foreach (Node n in graph.nodes) // graph.path 
                //{
                //    Gizmos.color = (n.walkable) ? Color.blue : Color.red;
                //    if (graph.path != null && graph.path.Contains(n))
                //        Gizmos.color = Color.white;
                //    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                //}

                Node currentNode = graph.getNodeFromPoint(transform.position);
                //Debug.Log("CAR INITIAL NODE: [" + currentNode.i + "," + currentNode.j + "]");
                
                
                
                Gizmos.color = (carInFront) ? Color.red : Color.cyan; // position of car
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                if ((debugOn) && (carId == debugThisCarId))
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawCube(transform.position, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                }

                //foreach (Node n in stopLines)
                //{
                //    Gizmos.color = Color.black;
                //    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));

                //}

                if (path != null)
                {
                    for (int i = 0; i < path.Count - 1; i++)
                    {
                        Gizmos.color = Color.black;
                        Gizmos.DrawLine(path[i].worldPosition, path[i + 1].worldPosition);
                    }
                    //Gizmos.color = Color.yellow;
                    //Gizmos.DrawCube(terrain_manager.myInfo.goal_pos, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                    //Gizmos.DrawCube(terrain_manager.myInfo.start_pos, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                }

            }

            
        }

        void stop()
        {
                m_Car.Move(0f, 0f, -1f, 1f);
        }

        void drive()
        {
            RaycastHit hit;
            Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, 50f);
            //Debug.Log(hit.distance);
            //Quaternion rotation = Quaternion.Euler(0, 90, 0);
            //Debug.DrawLine()


            Node node = graph.getNodeFromPoint(transform.position);
            Debug.DrawLine(transform.position, graph.nodes[node.i + 2, node.j].worldPosition, Color.blue);
            Vector3 car_position = m_Car.transform.position;

            // keep track of target position and velocity
            if (recovery > 0)
            { 
                recovery--; 
            }
            else
            {
                setAccelerationSteering(nextWaypoint);
                avoid_obstacles();
                

            }
            acceleration = Mathf.Clamp(acceleration, 0f, 0.5f);
            m_Car.Move(steering, acceleration, footbrake, handbrake);
            //if (had_hit)
            //{
            //    this.steering = 0f; 
            //    had_hit = false;

            //}
            //else 
            //{
            //    avoid_obstacles();
            //}




            Debug.DrawLine(car_position, path[nextWaypoint].worldPosition, Color.white);
            //Debug.DrawLine(transform.position, transform.position + car_velocity, Color.blue);
            //Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            //m_Car.Move(steering, acceleration, acceleration, 0f);

            if (Vector3.Distance(car_position, path[nextWaypoint].worldPosition) < 9)
            {
                nextWaypoint++;
            }
        }
        

        public void setAccelerationSteering(int waypointNumber)
        {
            float k_p = 0.3f;
            float k_d = 0.2f;


            Vector3 target_position = path[waypointNumber].worldPosition;
            target_position[1] = 0.1f;

            Vector3 position_error = target_position - transform.position;

            Vector3 car_position = m_Car.transform.position;


            Vector3 car_velocity = (car_position - oldCarPosition) / Time.fixedDeltaTime;

            Vector3 velocity_error = target_velocity - car_velocity;
            oldCarPosition = car_position;

            Vector3 correction_vector = k_p * position_error + k_d * velocity_error;

            float forward_velocity = Vector3.Dot(correction_vector, transform.forward);
            if (forward_velocity > 0)
            {
                this.acceleration = 1f;
                this.footbrake = 0f; 
            }
            else
            {
                this.acceleration = 0f;
                this.footbrake = -1f; // acceleration is clamped to 0,1. We reverse using the footbrake. 

            }

            steering = Vector3.Dot(correction_vector, transform.right);

        }

        private void CarInFront()
        {
            //float maxRange = terrainInfo.x_N;
            float maxRange = 7f;
            RaycastHit hit_forward;
            Vector3 forward = transform.TransformDirection(Vector3.forward);
            //Debug.DrawRay(transform.position+transform.up, forward * maxRange, Color.red);
            bool hit = Physics.Raycast(transform.position+transform.up, forward, out hit_forward, maxRange, layerMask);
            if (hit) // && (hit_forward.transform.name == "CarBlimp(Clone)"))
            {
                //Debug.DrawRay(transform.position+transform.up, forward * hit_forward.distance, Color.cyan);
                carInFront = true;
                //Debug.Log("I see a car");

                //Vector3 adjust = new Vector3(0,0,1f);
                //Vector3 adjust2 = new Vector3(1f, 0, 0);
                //Debug.DrawLine(hit_forward.point + adjust,hit_forward.point - adjust, Color.green);
                //Debug.DrawLine(hit_forward.point + adjust2, hit_forward.point - adjust2, Color.green);
                //Debug.DrawLine(transform.position, hit_forward.point, Color.green);

            }
            else
            {
                carInFront = false;
            }
            //            print(carInFront);
            
        }
        private List<Node> pathAdjustRightWall(List<Node> path)
        {
            List<Node> correctedPath = new List<Node>();

            foreach (Node n in path)
            {
                if (n.heading == 90)
                {
                    if ((!graph.nodes[n.i + 2, n.j].walkable) && (!graph.nodes[n.i - 1, n.j].walkable) && (graph.nodes[n.i + 1, n.j].walkable))
                    {
                        correctedPath.Add(graph.nodes[n.i + 1, n.j]);
                    }
                    else
                    {
                        correctedPath.Add(n);
                    }
                }
                else if (n.heading == 180)
                {
                    if ((!graph.nodes[n.i, n.j + 2].walkable) && (!graph.nodes[n.i, n.j - 1].walkable) && (graph.nodes[n.i, n.j + 1].walkable))
                    {
                        correctedPath.Add(graph.nodes[n.i, n.j + 1]);
                    }
                    else
                    {
                        correctedPath.Add(n);
                    }
                }
                else if (n.heading == 270)
                {
                    if ((!graph.nodes[n.i - 2, n.j].walkable) && (!graph.nodes[n.i + 1, n.j].walkable) && (graph.nodes[n.i - 1, n.j].walkable))
                    {
                        correctedPath.Add(graph.nodes[n.i - 1, n.j]);
                    }
                    else
                    {
                        correctedPath.Add(n);
                    }
                }
                else if (n.heading == 360 || n.heading == 0)
                {
                    if ((!graph.nodes[n.i, n.j - 2].walkable) && (!graph.nodes[n.i, n.j + 1].walkable) && (graph.nodes[n.i, n.j - 1].walkable))
                    {
                        correctedPath.Add(graph.nodes[n.i, n.j - 1]);
                    }
                    else
                    {
                        correctedPath.Add(n);
                    }
                }
                else
                {
                    correctedPath.Add(n);
                }
            }
            return correctedPath; 
        }


        private void avoid_obstacles()
        {
            RaycastHit forward_hit;
            RaycastHit backward_hit;
            RaycastHit right_hit;
            RaycastHit left_hit;
            RaycastHit fr_hit;
            RaycastHit fl_hit;
            RaycastHit br_hit;
            RaycastHit bl_hit;

            RaycastHit hit;

            Vector3 maxRange = carSize * 1.2f;
            //LayerMask onlyObstacles = ~(layerMask); // should give all layers except the one with the cars.
            LayerMask onlyObstacles = Physics.AllLayers;

            bool hit_forward = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out forward_hit, maxRange.z, onlyObstacles);
            bool hit_back = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back), out backward_hit, maxRange.z, onlyObstacles);
            bool hit_right = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.right), out right_hit, maxRange.x, onlyObstacles);
            bool hit_left = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.left), out left_hit, maxRange.x, onlyObstacles);
            bool hit_forward_right = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward + Vector3.right), out fr_hit, maxRange.z, onlyObstacles);
            bool hit_forward_left = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward + Vector3.left), out fl_hit, maxRange.z, onlyObstacles);
            bool hit_back_right = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back + Vector3.right), out br_hit, maxRange.z, onlyObstacles);
            bool hit_back_left = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back + Vector3.left), out bl_hit, maxRange.z, onlyObstacles);


            if ((hit_forward) && (!hit_right) && (!hit_left))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * forward_hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                Debug.Log("Hit forward only. Frontal collision, distance: " + forward_hit.distance + " CarId: " + carId);

                if (forward_hit.distance > 5){
                   //this.acceleration *= 0.5f;
                   if (Mathf.Abs(this.steering) < 0.1) {
                        this.steering = this.steering * 100f; 
                    }
                   //this.footbrake = this.footbrake < 0.1f ? 0.5f : this.footbrake * 2;
                   Debug.Log("Forward hit distance > 4. CarId: " + carId);
                }
                else
                {
                    Debug.Log("Forward hit collision STOP. CarId: " + carId);
                    this.acceleration = 0;
                    this.footbrake = -1; // reverse the car
                    if (this.steering < 0)
                    {
                        this.steering = 1;
                    }
                    else
                    {
                        this.steering = -1;
                    }
                    had_hit = true; // oops I hit something.
                    recovery = 5;

                }


                //this.steering *= -1;
                this.handbrake = 0;
            }
            else if (hit_forward_right)
            {
                if (fr_hit.distance > 3.5)
                {
                    this.steering = -1f;
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward+Vector3.right) * fr_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    Debug.Log("Forward right hit distance > 3. Distance: " + fr_hit.distance + " CarId: " + carId);
                }
                else
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward+Vector3.right) * fr_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    this.footbrake = -1;
                    this.acceleration = 0;
                    this.steering = 1;
                    Debug.Log("Forward right hit collision. CarId: " + carId);
                    had_hit = true;
                    recovery = 5;


                }

            }
            else if (hit_forward_left)
            {
                if (fl_hit.distance > 3.5)
                {
                    this.steering = 1f;
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward + Vector3.left) * fl_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    Debug.Log("Forward left hit distance > 3. CarId: " + carId);

                }
                else
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward+Vector3.left) * fl_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    this.footbrake = -1;
                    this.acceleration = 0;
                    this.steering =-1;
                    Debug.Log("Forward left hit collision. CarId: " + carId);
                    had_hit = true;
                    recovery = 5;

                }

            }

            else if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back), out hit, maxRange.z, onlyObstacles)) // The last time we used this THIS DID NOT WORK
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.back) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.acceleration = 1;
                this.footbrake = 0;
                Debug.Log("Back collision");
                had_hit = true;
                recovery = 5;


            }

            else if (hit_right)
            {
                if (right_hit.distance > 1.5)
                {
                    this.steering = -1f;
                    this.acceleration *= 0.5f;

                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.right) * right_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    Debug.Log("Right hit distance > 3. CarId: " + carId);
                }
                else // <3
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.right) * hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    this.steering = 1f;
                    this.acceleration = 0f;
                    this.footbrake = -1f;
                    had_hit = true;
                    recovery = 5;

                    Debug.Log("Right hit collision. CarId: " + carId);
                }
                //this.acceleration *= 0.7f;
                //this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                //this.steering += -0.8f;
                //Debug.Log("Right collision: " + carId);
                //had_hit = true; // oops I hit something.


            }

            else if (hit_left)
            {
                if (left_hit.distance > 1.5)
                {
                    this.steering = 1f;
                    this.acceleration *= 0.5f;
                    this.footbrake = 0f;

                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.left) * left_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    Debug.Log("Left hit distance > 3. CarId: " + carId);
                }
                else
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.left) * left_hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    this.acceleration = 0;
                    this.steering = -1f;
                    this.footbrake = -1f; 
                    Debug.Log("Left hit collision. CarId: " + carId);
                    had_hit = true; // oops I hit something.
                    recovery = 5;

                }
                //this.acceleration *= 0.7f;
                //this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                //this.steering += 0.8f;
                Debug.Log("Left collision: " + carId);

                
            }
            else if (had_hit)
            {
                this.acceleration = 1f;
                this.footbrake = 0f;
                this.steering = 0f; //or *-1
                Debug.Log("Headed forward again after a collision: " + carId);
                recovery = 5;
                had_hit = false;
            }
            //if (!had_hit && !curve_approaching)
            //{
            //    this.acceleration *= 1.25f;
            //    Debug.Log("Not hit speed");
            //}

            //if (!had_hit && m_Car.CurrentSpeed < 1f || had_hit_backward) // The car current speed can be <1f if they have a car in front of them, so this is making a problem. It says if I haven't hit anything but my speed is below 1, then I probably backed into something. 
            //{
            //    had_hit_backward = true;
            //    this.acceleration = 1;
            //    this.footbrake = 0;
            //    this.handbrake = 0;
            //    this.steering *= 1;
            //    if (m_Car.CurrentSpeed > 10f)
            //        had_hit_backward = false;
            //}
        }

        void setCarId()
        {
            for (int i = 0; i < friends.Length; i++)
            {
                if (m_Car.transform.position == friends[i].transform.position)
                {
                    //car_id = convexCovers.Count;
                    carId = i;
                }
            }
        }

        public int get_closest_node(Vector3 position, List<Node> path, int current_index)
        {
            //Debug.Log("Finding the closest node to " + position);
            int closest = -1;
            float distance = 1000000000;
            position = new Vector3(position.x, 0, position.z);
            int range = 10;
            for (int i = (int)Mathf.Clamp(current_index - range, 0, path.Count); i < Math.Min(path.Count, current_index + 10); i++)
            {
                Vector3 to_compare = new Vector3(path[i].x_pos, 0, path[i].z_pos);
                float new_distance = Vector3.Distance(position, to_compare);
                if (new_distance <= distance)
                {
                    distance = new_distance;
                    closest = i;
                    //Debug.Log("Closest node at index i:" + i + " distance: " + distance + " and position " + path[i].worldPosition);
                }
            }
            return closest;
        }
    }
}
