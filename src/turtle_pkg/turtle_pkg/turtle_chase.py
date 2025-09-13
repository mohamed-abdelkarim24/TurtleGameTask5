#!/usr/bin/env python3

import rclpy
import math
import random
from rclpy.node import Node
from turtlesim.srv import Spawn , Kill
from turtlesim.msg import Pose
from std_msgs.msg import Int32
from functools import partial


class turtle_chase(Node):
    def __init__(self):
       
       self.score = 0
       self.playerpose = None
       self.enemypose = {}
       self.msg = None
       
       super().__init__('turtle_chase')
       self.score_ = self.create_publisher(Int32, '/score', 10)
       

       self.spawnclient = self.create_client(Spawn, 'spawn')
       self.killclient = self.create_client(Kill,'kill')

       self.playerSub = self.create_subscription(Pose , 'turtle1/pose' , self.player_callback ,10)
      # self.enemysub = self.create_subscription(Pose , 'enemy1/pose' , self.enemy1_callback , 10)
      # self.enemy1sub = self.create_subscription(Pose , 'enemy2/pose' , self.enemy2_callback , 10)
      # self.enemy1sub = self.create_subscription(Pose , 'enemy3/pose' , self.enemy3_callback , 10)
       
       self.enemyspawn('enemy1')
       self.enemyspawn('enemy2')
       self.enemyspawn('enemy3')
       self.timer = self.create_timer(0.0001, self.check_collisions)
       self.timer = self.create_timer(0.001, self.score_callback)
    
    def score_callback(self):
        msg = Int32()
        msg.data = self.score
        self.score_.publish(msg)


    def player_callback(self , pose : Pose):
        self.playerpose = pose
        
    def enemysubscribe(self , name):
        topic = f"{name}/pose"
        self.enemysub = self.create_subscription(Pose , topic , self.enemy_callback, 10)
    
    
    def enemy_callback(self , msg):
        self.msg = msg
    

    def enemyspawn(self , enemy_name):
        
        request = Spawn.Request()
        request.x = random.uniform(0.5, 10.5)
        request.y = random.uniform(0.5, 10.5)
        request.theta = random.uniform(-math.pi, math.pi)
        request.name = enemy_name
        
        future = self.spawnclient.call_async(request)
           
        future.add_done_callback(partial(self.callback_spawn , request = request))

    def callback_spawn(self , future , request):
        response = future.results()
        self.enemysubscribe(request.name)
        self.enemypose[request.name] = self.msg

    def enemykill(self, enemy_name):

        request = Kill.Request()
        request.name = enemy_name
        
        future = self.killclient.call_async(request)

        future.add_done_callback(partial(self.callback_kill , request = request))

    def callback_kill(self , future , request):
        response = future.result() 
        self.get_logger().info(f"Successfully killed {request.name}")
        self.score +=1

            

    def check_collisions(self):
        if self.playerpose is not None :
            for name , pose in list(self.enemypose.items()):
                dist = math.sqrt((self.playerpose.x - pose.x)**2 + (self.playerpose.y - pose.y)**2)

                if dist < 0.5 :
                    self.get_logger().info(f"{name} is hit")
                    self.enemykill(name)
                    self.enemyspawn(name)
        else : raise Exception("ERROR")


                
                
            
    
            
    