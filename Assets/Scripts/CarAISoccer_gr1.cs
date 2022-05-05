using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Panda;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAISoccer_gr1 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public string friend_tag;
        public GameObject[] enemies;
        public string enemy_tag;

        public GameObject own_goal;
        public GameObject other_goal;
        public GameObject ball;
        public GameObject ball_spawn_point;


        public float dist;
        public float maxKickSpeed = 40f;
        public float lastKickTime = 0f;

        PandaBehaviour myPandaBT;

        float passVelocity;
        float shootVelocity;
        float dribbleVelocity;
        bool stop = false;

        // Driving helpers
        float steering;
        float acceleration;
        float footbrake;
        float handbrake;
        private Vector3 target_velocity;
        private Vector3 oldCarPosition;
        private Vector3 oldTargetPosition;
        bool collision = false;

        static bool goalie_set = false;
        bool is_goalie = false;
        static bool forward_set = false;
        bool is_forward = false;
        static bool midfield_set = false;
        bool is_midfield = false;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            myPandaBT = GetComponent<PandaBehaviour>();



            friend_tag = gameObject.tag;
            if (friend_tag == "Blue")
                enemy_tag = "Red";
            else
                enemy_tag = "Blue";

            friends = GameObject.FindGameObjectsWithTag(friend_tag);
            enemies = GameObject.FindGameObjectsWithTag(enemy_tag);

            ball = GameObject.FindGameObjectWithTag("Ball");

            steering = 0f;
            acceleration = 0f;
            footbrake = 0f;
            handbrake = 0f;

            shootVelocity = maxKickSpeed;
            dribbleVelocity = maxKickSpeed / 5;
            passVelocity = maxKickSpeed / 2;

            ball_spawn_point = ball.GetComponent<GoalCheck>().ball_spawn_point;

            assignPositions();
            startPositions();

        }
        [Task]
        bool IsBallOutOfBounds()
        {
            float out_of_bounds = 150f; //taken from GoalCheck
            return ((transform.position - ball_spawn_point.transform.position).magnitude > out_of_bounds);
        }
        [Task]

        bool IsMidfield()
        {
            return is_midfield;
        }

        [Task]
        bool IsForward()
        {
            return is_forward;
        }
        
        [Task]
        bool IsBallBetweenUsAndGoal()
        {
            bool ballbetweenUsAndGoal = Vector3.Distance(ball.transform.position, other_goal.transform.position) < Vector3.Distance(transform.position, other_goal.transform.position);
            return ballbetweenUsAndGoal;
        }

        [Task]
        bool IsBallCloserThan(float distance)
        {
            return ((ball.transform.position - transform.position).magnitude < distance);
        }

        [Task]
        bool IsBallCloseToGoal(float distance)
        {
            return ((ball.transform.position - other_goal.transform.position).magnitude < distance);
        }

        [Task]
        bool IsGoalie()
        {
            return is_goalie;
            //if (friends == null) 
            //{ 
            //    Debug.Log("friends[0] is null"); 
            //}
            //GameObject goalie = friends[0];
            //float shortestDistance = Mathf.Infinity;
            //foreach (GameObject friend in friends)
            //{
            //    float distnaceToGoal = Vector3.Distance(own_goal.transform.position, friend.transform.position);
            //    if (distnaceToGoal < shortestDistance)
            //    {
            //        goalie = friend;
            //    }
            //}
            //if (goalie.transform.position == transform.position)
            //{
            //    return true;
            //}
            //else
            //{
            //    return false;
            //}
        }
        [Task]
        bool IsNearGoal(float distance)
        {
            return ((transform.position - own_goal.transform.position).magnitude < distance);
        }

        
        //bool IsForward()
        //{
        //    GameObject forward = friends[0];
        //    float shortestDistance = Mathf.Infinity;
        //    foreach (GameObject friend in friends)
        //    {
        //        float distnaceToGoal = Vector3.Distance(other_goal.transform.position, friend.transform.position);
        //        if (distnaceToGoal < shortestDistance)
        //        {
        //            forward = friend;
        //        }
        //    }
        //    if (forward.transform.position == transform.position)
        //    {
        //        return true;
        //    }
        //    else
        //    {
        //        return false;
        //    }
        //}

        [Task]
        void Shoot()
        {
            Vector3 direction = new Vector3();

            Vector3 goalsize = other_goal.GetComponent<BoxCollider>().size;
            goalsize[0] = 0f;
            goalsize[1] = 0f;
            if (!Physics.Linecast(transform.position, other_goal.transform.position)) {
                direction = (other_goal.transform.position - transform.position).normalized;

            }
            else if (!Physics.Linecast(transform.position, other_goal.transform.position + goalsize/2))
            {
                direction = ((other_goal.transform.position + goalsize/2) - transform.position).normalized;

            }
            else
            {
                direction = ((other_goal.transform.position - goalsize / 2) - transform.position).normalized;

            }
            Vector3 shoot = direction * shootVelocity;
            if (CanKick())
            {
                Debug.Log("Shooting");

                KickBall(shoot);
            }
        }


        void assignPositions()
        {
                int count_near = 0;
                float own_distance = Vector3.Distance(own_goal.transform.position, this.transform.position);
                foreach (GameObject friend in friends)
                {
                    float friendToGoal = Vector3.Distance(own_goal.transform.position, friend.transform.position);
                    if (friendToGoal < own_distance)
                    {
                        count_near++; //how many friends are closer to the goal than you?  
                    }
                }
                if ((count_near == 0) && (goalie_set == false)) //you are closest to goal 
                {
                    is_goalie = true;
                    is_midfield = false;
                    is_forward = false;
                    goalie_set = true;
                }
                else if ((midfield_set == false))//only one friend is closer to goal than you 
                {
                    midfield_set = true;
                    is_midfield = true;
                    is_forward = false;
                    is_goalie = false;
                }
                else if (forward_set == false)
                {
                    forward_set = true;
                    is_forward = true;
                    is_goalie = false;
                    is_midfield = false;
                }

        }

        [Task]
        void MoveToGoal()
        {
            MoveCar(own_goal.transform.position);
            // get between the ball and the goal
        }

        [Task]
        void startPositions()
        {
            if (is_goalie)
            {
                MoveCar(own_goal.transform.position);
            }
            else if (is_midfield)
            {
                MoveCar(ball_spawn_point.transform.position);
            }
            else // Forward position
            {
                Vector3 forwardStartPoint = ball_spawn_point.transform.position + Vector3.forward * 2 + Vector3.right * 2;
                MoveCar(Vector3.forward);
            }
        }

        [Task]
        void GoToCentre() 
        {
            MoveCar(ball_spawn_point.transform.position);
        }
        [Task]
        void BeGoalie()
        {
            //Moves the car in z position only, in other words it stays in the goal and moves along it. 
            // TODO figure out how to park the car sideways and only drive forward and backward. 
            // New routine for moving the goalie, MoveGoalie?
            // Figure out how to park the car north/south
            Vector3 ball_position = ball.transform.position; // (x, y, z)
            Vector3 current = transform.position;
            Mathf.Clamp(ball_position.z, own_goal.transform.position.z + (own_goal.GetComponent<BoxCollider>().size.z / 2), own_goal.transform.position.z - (own_goal.GetComponent<BoxCollider>().size.z / 2));
            Vector3 new_goalie_position = new Vector3(current.x, current.y, ball_position.z);
            MoveCar(new_goalie_position);

            Debug.DrawLine(transform.position, new_goalie_position, Color.red);
            // get between the ball and the goal
        }
        [Task]
        bool SafeToShoot()
        {
            if (Physics.Linecast(transform.position, other_goal.transform.position)) { return false; }
            else { return true; }
        }

        [Task]
        bool SafeToPass()
        {
            GameObject nearest_friend = friends[0];
            float shortestDistance = Mathf.Infinity;

            foreach (GameObject friend in friends)
            {
                float distanceToFriend = Vector3.Distance(transform.position, friend.transform.position);
                //do not want to pass ourselves 
                if (distanceToFriend > 5f)
                {
                    if (distanceToFriend < shortestDistance)
                    {
                        nearest_friend = friend;
                        shortestDistance = distanceToFriend;
                    }
                }
                else
                {
                    nearest_friend = other_goal; //incase all friends are too close 

                }

            }
            if (nearest_friend.transform.position == other_goal.transform.position)
            {
                return false;
            }
            if (Physics.Linecast(transform.position, nearest_friend.transform.position))
            {
                return false;
            }
            if ((transform.position-other_goal.transform.position).magnitude < (nearest_friend.transform.position - other_goal.transform.position).magnitude) //don't pass backward
            {
                return false;
            }
            else { return true; }
        }

            [Task]
        void PassBall()
        {
            GameObject nearest_friend = friends[0];
            float shortestDistance = Mathf.Infinity;

            foreach (GameObject friend in friends)
            {
                float distanceToFriend = Vector3.Distance(transform.position, friend.transform.position);
                //do not want to pass ourselves 
                if (distanceToFriend > 5f)
                {
                    if (distanceToFriend < shortestDistance)
                    {
                        nearest_friend = friend;
                        shortestDistance = distanceToFriend;
                    }

                }
                else
                {
                    nearest_friend = other_goal; //incase all friends are too close 
                }
            }
            Vector3 direction = new Vector3();
            direction = (nearest_friend.transform.position - transform.position).normalized;
            Vector3 pass = direction * passVelocity;
            if (CanKick())
            {
                //Debug.Log("Passing");

                KickBall(pass);
            }
        }

        
        [Task]
        void Dribble()
        {
            //Figure out the direction to kick here and test

            Vector3 direction = new Vector3();
            direction = (other_goal.transform.position - transform.position).normalized;
            Vector3 dribble = direction * dribbleVelocity;
            if (CanKick())
            {
                Debug.Log("Dribbling");
                KickBall(dribble);
            }
            MoveCar(ball.transform.position);
        }

        [Task]
        void ChaseMidfield()
        {
            Vector3 direction = (ball.transform.position - other_goal.transform.position);

            Vector3 directionNorm = direction.normalized;
            float offset = direction.magnitude * 1.1f;
            Vector3 destination = other_goal.transform.position + directionNorm * offset + Vector3.up;
            destination[1] = 0.1f;
            Debug.DrawLine(transform.position, destination, Color.cyan);

            MoveCar(destination);
        }
        [Task]
        void ChaseForward()
        {
            Vector3 direction = (ball.transform.position - other_goal.transform.position);

            Vector3 directionNorm = direction.normalized;
            float offset = direction.magnitude * 1.1f;
            Vector3 destination = other_goal.transform.position + directionNorm * offset;
            destination[1] = 0.1f;
            Debug.DrawLine(transform.position, destination, Color.green);

            MoveCar(destination);
        }

        [Task]
        bool IsItOurBall() 
        { 
            
            return false; 
        }
        [Task]
        bool IsItOnOurSideOfThePitch() 
        {
            return ((other_goal.transform.position - ball.transform.position).magnitude > (own_goal.transform.position - ball.transform.position).magnitude);
        }

        [Task]
        void GetBehindBallMidfield()
        {
            //simple function to drive around to the other side of the ball. first, find the point to drive to.
            Vector3 direction = (ball.transform.position - other_goal.transform.position);
            Debug.DrawRay(other_goal.transform.position, direction, Color.black);

            Vector3 directionNorm = direction.normalized;
            float offset = direction.magnitude * 1.2f;
            Vector3 destination = other_goal.transform.position + directionNorm * offset ;
            destination[1] = 0.1f;
            Debug.DrawLine(transform.position, destination, Color.red);
            MoveCar(destination + Vector3.left + Vector3.back);
            m_Car.Move(1f, 1f, 1f, 0f);
            MoveCar(destination);
        }

        [Task]
        void GetBehindBallForward()
        {

            // How about this for getting behind:
            // Set a destination that is vector3 right and forward, then do a left turn to get behind. the midfielder can go the other direction.
            //simple function to drive around to the other side of the ball. first, find the point to drive to.
            Vector3 direction = (ball.transform.position - other_goal.transform.position);
            Debug.DrawRay(other_goal.transform.position, direction, Color.cyan);

            Vector3 directionNorm = direction.normalized;
            float offset = direction.magnitude * 1.1f;
            Vector3 destination = other_goal.transform.position + directionNorm * offset;
            destination[1] = 0.1f;
            Debug.DrawLine(transform.position, destination, Color.yellow);
            MoveCar(destination + Vector3.right + Vector3.forward);
            m_Car.Move(-1f, 1f, 1f, 0f);
            MoveCar(destination);

        }
        [Task]

        private bool CanKick()
        {
            dist = (transform.position - ball.transform.position).magnitude;
            return dist < 7f && (Time.time - lastKickTime) > 0.5f;
        }

        private void KickBall(Vector3 velocity)
        {
            // impulse to ball object in direction away from agent
            if (CanKick())
            {
                velocity.y = 0f;
                if (Vector3.Dot(velocity, ball.transform.position - transform.position) < 0f)
                {
                    print("Must kick away from agent");
                    velocity = Vector3.zero;
                }
                Rigidbody rb = ball.GetComponent<Rigidbody>();
                rb.AddForce(velocity, ForceMode.VelocityChange);
                lastKickTime = Time.time;
                print("ball was kicked ");

            }

        }

        private void setNewRoles()
        {
            CarAISoccer_gr1 forwardFriend = friends[0].GetComponent<CarAISoccer_gr1>();

            if (is_goalie) { return; } //if you're the goalie, don't give yourself a new role
            if ((forward_set == true)  && (midfield_set == true)) { return; } // if roles have been set by the other forward, don't give yourself a new role
            foreach (GameObject friend in friends) // find the other forward and store it in forwardFriend
            {
                forwardFriend = friend.GetComponent<CarAISoccer_gr1>();
                if ((friend.transform.position == transform.position) || (friend.GetComponent<CarAISoccer_gr1>().is_goalie==true))
                { 
                    continue; 
                }
                else //check these two for switching roles
                {
                    break;
                }
            }
            if ((ball.transform.position - transform.position).magnitude < (ball.transform.position - forwardFriend.transform.position).magnitude)
            {
                is_forward = true;
                is_midfield = false;
                forward_set = true;
                forwardFriend.is_midfield = true;
                forwardFriend.is_forward = false;
                midfield_set = true;
            }
            else
            {
                is_midfield = true;
                is_forward = false;
                midfield_set = true;
                forwardFriend.is_forward = true;
                forwardFriend.is_midfield = false;
                forward_set = true;
            }
        }

        private void collisionCheck()
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

            Vector3 carSize = new Vector3(2.43f, 0.41f, 4.47f);

            Vector3 maxRange = carSize * 1.2f;
            //LayerMask onlyObstacles = ~(layerMask); // should give all layers except the one with the cars.
            LayerMask cubes = 1 << 9;
            LayerMask otherCars = 1 << 6;
            LayerMask onlyObstacles = cubes;

            bool hit_forward = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out forward_hit, maxRange.z, onlyObstacles);
            bool hit_back = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back), out backward_hit, maxRange.z, onlyObstacles);
            bool hit_right = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.right), out right_hit, maxRange.x, onlyObstacles);
            bool hit_left = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.left), out left_hit, maxRange.x, onlyObstacles);
            bool hit_forward_right = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward + Vector3.right), out fr_hit, maxRange.z, onlyObstacles);
            bool hit_forward_left = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward + Vector3.left), out fl_hit, maxRange.z, onlyObstacles);
            bool hit_back_right = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back + Vector3.right), out br_hit, maxRange.z, onlyObstacles);
            bool hit_back_left = Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back + Vector3.left), out bl_hit, maxRange.z, onlyObstacles);

            if ((hit_forward) ||  (hit_forward_right) || (hit_forward_left)) 
            {
                if (!hit_left)
                {
                    steering = 1f;
                }
                else if (!hit_right)
                {
                    steering = -1f;
                }
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * forward_hit.distance;
                Vector3 closestObstacleInFront2 = transform.TransformDirection(Vector3.forward + Vector3.right) * fr_hit.distance;
                Vector3 closestObstacleInFront3 = transform.TransformDirection(Vector3.forward + Vector3.left) * fl_hit.distance;
                
                if (hit_forward) { Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow); }
                if (hit_forward_right) { Debug.DrawRay(transform.position, closestObstacleInFront2, Color.yellow); }
                if (hit_forward_left) { Debug.DrawRay(transform.position, closestObstacleInFront3, Color.yellow); }

                acceleration = 0f;
                footbrake = -1f;
                handbrake = 0f;
                collision=true; }
            else if ((hit_forward_right) && (hit_back_right) && (!hit_forward))
            {
                acceleration = 1f;
                steering = -1f;
                footbrake = 0f;
                handbrake = 0f;
            }
            else if ((hit_forward_left) && (hit_back_left) && (!hit_forward))
            {
                acceleration = 1f;
                steering = 1f;
                footbrake = 0f;
                handbrake = 0f;
            }
            else if ((hit_back) || (hit_back_right) || (hit_back_left)) {

                if (!hit_left)
                {
                    steering = -1f;
                }
                else if (!hit_right)
                {
                    steering = 1f;
                }
                
                acceleration = 1f;
                handbrake = 0f;
                footbrake = 0f;
                collision=true; 
            
            }
            else if ((hit_left))
            {
                acceleration = -acceleration;
                if (acceleration < 0f)
                {
                    steering = -1f;
                }
                else { steering = 1f; }
                handbrake = 0f;
                footbrake = 0f;
                collision = true;
            }
            else if ((hit_right))
            {
                acceleration = -acceleration;
                if (acceleration < 0f)
                {
                    steering = 1f;
                }
                else { steering = -1f; }
                handbrake = 0f;
                footbrake = 0f;

                collision = true;
            }
            else
            {
                collision=false;
            }
            Debug.Log("Collision");

        }

        private void MoveCar(Vector3 destination)
        {
            if (stop == true)
            {
                m_Car.Move(0f, 0f, 1f, 0f);
                return;
            }

            collisionCheck();
            if (!collision)
            {

                float arrivalRange = 3f;
                float distance = Vector3.Distance(transform.position, destination);
                if (distance > arrivalRange)
                {
                    float k_p = 0.3f;
                    float k_d = 0.2f;
                    destination[1] = 0.1f;

                    //Vector3 directionToMove = (destination - transform.position).normalized;
                    //Debug.Log("Dir to move: " + directionToMove);

                    //float dot = Vector3.Dot(transform.forward, directionToMove);
                    //float steeringAngle = Vector3.SignedAngle(transform.forward, directionToMove, Vector3.up);
                    //steeringAngle = Mathf.Clamp(steeringAngle, -25, 25);
                    //steering = steeringAngle / m_Car.m_MaximumSteerAngle;

                    // Figure out when/how to use Dubins.
                    //if (dot >= 0)
                    //{
                    //    acceleration = 1f;
                    //}
                    //else
                    //{
                    //    acceleration = 0f;
                    //    footbrake = -1f; // back up
                    //}

                    Vector3 position_error = destination - transform.position;

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
                        this.handbrake = 0f;

                    }
                    else
                    {
                        this.acceleration = 0f;
                        this.footbrake = -1f; // acceleration is clamped to 0,1. We reverse using the footbrake. 
                        this.handbrake = 0f;
                    }

                    steering = Vector3.Dot(correction_vector, transform.right);

                }
                //else
                //{
                //    // Stop the car
                //    steering = 0;
                //    acceleration = 0f;
                //    footbrake = -1f;
                //    handbrake = 1f;
                //    Debug.Log("Stopping");

                //}
                

            }
            Debug.Log("Moving or stopping: " + steering + " " + acceleration + " " + footbrake + " " + handbrake);

            m_Car.Move(steering, acceleration, footbrake, handbrake);

            if (m_Car.CurrentSpeed < 0.1f)
            {
                Debug.Log("I think I'm stuck. I'll just go forward");
                m_Car.Move(0f, 1f, 0f, 0f);
            }
            if (m_Car.CurrentSpeed < 0.1f)
            {
                Debug.Log("I think I'm still stuck, I'll try backward");
                m_Car.Move(0f,0f,-1f,0f);
            }
        }


        private void Update()
        {
            myPandaBT.Reset();
            myPandaBT.Tick();
            forward_set = false;
            midfield_set = false;
            setNewRoles();
        }
        /*private void FixedUpdate()
        {


            // Execute your path here
            // ...

            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / friends.Length;
            //Vector3 direction = (avg_pos - transform.position).normalized;
            Vector3 direction = (ball.transform.position - transform.position).normalized;

            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            float steering = 0f;
            float acceleration = 0;

            if (is_to_the_right && is_to_the_front)
            {
                steering = 1f;
                acceleration = 1f;
            }
            else if (is_to_the_right && !is_to_the_front)
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right && is_to_the_front)
            {
                steering = -1f;
                acceleration = 1f;
            }
            else if (!is_to_the_right && !is_to_the_front)
            {
                steering = 1f;
                acceleration = -1f;
            }

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, ball.transform.position, Color.black);
            Debug.DrawLine(transform.position, own_goal.transform.position, Color.green);
            Debug.DrawLine(transform.position, other_goal.transform.position, Color.yellow);
            Debug.DrawLine(transform.position, friends[0].transform.position, Color.cyan);
            Debug.DrawLine(transform.position, enemies[0].transform.position, Color.magenta);

            if (CanKick())
            {
                Debug.DrawLine(transform.position, ball.transform.position, Color.red);
                //KickBall(maxKickSpeed * Vector3.forward);
            }

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            //m_Car.Move(0f, -1f, 1f, 0f);

            // this is how you kick the ball (if close enough)
            // Note that the kick speed is added to the current speed of the ball (which might be non-zero)
            Vector3 kickDirection = (other_goal.transform.position - transform.position).normalized;

            // replace the human input below with some AI stuff
            if (Input.GetKeyDown("space"))
            {
                KickBall(maxKickSpeed * kickDirection);
            }
        }*/
    }
}
