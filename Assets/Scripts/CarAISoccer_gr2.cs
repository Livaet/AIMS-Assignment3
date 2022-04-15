﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAISoccer_gr2 : MonoBehaviour
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

        public float dist;
        public float maxKickSpeed = 40f;
        public float lastKickTime = 0f;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
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


            // Plan your path here
            // ...
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

        private void FixedUpdate()
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
            Vector3 direction = (own_goal.transform.position - transform.position).normalized;

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
        }
    }
}
