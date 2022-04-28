using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Panda; 


[RequireComponent(typeof(DroneController))]
public class DroneAISoccer_blue : MonoBehaviour
{
    private DroneController m_Drone; // the drone controller we want to use
    float max_accel;

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public GameObject[] friends;
    public string friend_tag;
    public GameObject[] enemies;
    public string enemy_tag;

    public GameObject own_goal;
    public GameObject other_goal;
    public GameObject ball;

    public float dist;
    public float maxKickSpeed = 40f;
    public float lastKickTime = 0f;

    PandaBehaviour myPandaBT;

    float shootVelocity;
    float dribbleVelocity;
    bool stop = false; 

    float h_accel = 0f; 
    float v_accel = 0f; 

    private void Start()
    {
        myPandaBT = GetComponent<PandaBehaviour>();
        // get the car controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


        // note that both arrays will have holes when objects are destroyed
        // but for initial planning they should work
        friend_tag = gameObject.tag;
        if (friend_tag == "Blue")
            enemy_tag = "Red";
        else
            enemy_tag = "Blue";

        friends = GameObject.FindGameObjectsWithTag(friend_tag);
        enemies = GameObject.FindGameObjectsWithTag(enemy_tag);

        ball = GameObject.FindGameObjectWithTag("Ball");

        shootVelocity = 10f;
        dribbleVelocity = 5f;

        max_accel = m_Drone.max_acceleration;
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
        return ((ball.transform.position - transform.position).magnitude < distance );
    }

    [Task]
    bool IsBallCloseToGoal(float distance)
    {
        return ((ball.transform.position - other_goal.transform.position).magnitude < distance);
    }
    [Task]
    bool IsGoalie()
    {
        GameObject goalie = friends[0]; 
        float shortestDistance = Mathf.Infinity; 
        foreach (GameObject friend in friends)
        {
            float distnaceToGoal = Vector3.Distance(own_goal.transform.position, friend.transform.position);
            if (distnaceToGoal < shortestDistance)
            {
                goalie = friend; 
            }
        }
        if (goalie.transform.position == transform.position)
        {
            return true; 
        }
        else
        {
            return false; 
        }
    }

    [Task]
    bool IsForward()
    {
        GameObject forward = friends[0];
        float shortestDistance = Mathf.Infinity;
        foreach (GameObject friend in friends)
        {
            float distnaceToGoal = Vector3.Distance(other_goal.transform.position, friend.transform.position);
            if (distnaceToGoal < shortestDistance)
            {
                forward = friend;
            }
        }
        if (forward.transform.position == transform.position)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    [Task]
    void Shoot()
    {
        Vector3 direction = new Vector3();
        direction = (other_goal.transform.position - transform.position).normalized;
        Vector3 shoot = direction * shootVelocity;
        if (CanKick())
        {
            Debug.Log("Shooting");

            KickBall(shoot);
        }
    }

    [Task]
    void Defend()
    {
        MoveDrone(own_goal.transform.position);
        // get between the ball and the goal
    }

    [Task]
    void Dribble()
    {
        //Figure out the direction to kick here and test

        Vector3 direction = new Vector3();
        direction = (other_goal.transform.position - transform.position).normalized;
        Vector3 dribble = direction * dribbleVelocity;
        if (CanKick()) {
            Debug.Log("Dribbling");
            KickBall(dribble);        
        }
    }

    [Task]
    void Chase()
    {
        Vector3 direction = (ball.transform.position - other_goal.transform.position);

        Vector3 directionNorm = direction.normalized;
        float offset = direction.magnitude * 1.1f;
        Vector3 destination = other_goal.transform.position + directionNorm * offset;
        destination[1] = 0.1f;
        Debug.DrawLine(transform.position, destination, Color.red);

        MoveDrone(destination);
    }
    [Task]
    void GetBehindBall()
    {
        //simple function to drive around to the other side of the ball. first, find the point to drive to.
        Vector3 direction = (ball.transform.position - other_goal.transform.position);
        Debug.DrawRay(other_goal.transform.position, direction, Color.cyan);

        Vector3 directionNorm = direction.normalized;
        float offset = direction.magnitude * 1.1f;
        Vector3 destination = other_goal.transform.position + directionNorm * offset;
        destination[1] = 0.1f;
        Debug.DrawLine(transform.position, destination, Color.red);
        MoveDrone(destination);
    }

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
            Rigidbody rb = ball.GetComponent<Rigidbody>();
            rb.AddForce(velocity, ForceMode.VelocityChange);
            lastKickTime = Time.time;
            print("ball was kicked ");

        }

    }

    //Go to the next node based on the direction. Then, as soon as you are in range, you start slowing down. When you stop (regardless of where you are)
    //start heading to the next node.

    private void MoveDrone(Vector3 destination)
    {
        if (stop==true)
        {
            m_Drone.Move(0, 0);
            return;
        }

        float arrivalRange = 1.5f;
        float distance = Vector3.Distance(transform.position, destination);

        Vector3 directionToMove = (destination - transform.position).normalized;
        Debug.Log("Dir to move: " + directionToMove);
        
        if (distance > 10)
        {
            h_accel = max_accel * directionToMove.x;
            v_accel = max_accel * directionToMove.z;
        }
        else
        {
            h_accel = max_accel/2f * directionToMove.x;
            v_accel = max_accel/2f * directionToMove.z;
        }


        m_Drone.Move(h_accel, v_accel);

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



        // this is how you control the agent
        m_Drone.Move_vect(direction);

        // this is how you kick the ball (if close enough)
        // Note that the kick speed is added to the current speed of the ball (which might be non-zero)
        Vector3 kickDirection = (other_goal.transform.position - transform.position).normalized;

        // replace the human input below with some AI stuff
        if (Input.GetKeyDown("space"))
        {
            KickBall(maxKickSpeed * kickDirection);
        }

    }
    */

    private void Update()
    {
        myPandaBT.Reset();
        myPandaBT.Tick();
    }
}

