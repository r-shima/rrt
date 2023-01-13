# Rapidly-Exploring Random Tree
I implemented an RRT algorithm in Python. This project consists of two parts: Simple RRT and RRT with Obstacles.

## Simple RRT
In the first part, I implemented an RRT in a two-dimensional domain, D = [0, 100] x [0, 100], with no obstacles. The program runs for 800 iterations. Below is a demonstration:

<p align="center">
    <img align="center" src="https://github.com/r-shima/rrt/blob/main/images/simple_rrt.gif">
</p>

## RRT with Obstacles
In the second part, I implemented an RRT in a two dimensional domain, D = [0, 100] x [0, 100], with 40 circular obstacles. The program runs for 2000 iterations. The start locations, goal locations, and obstacle size and locations are randomly generated. Below is a demonstration:

<p align="center">
    <img align="center" src="https://github.com/r-shima/rrt/blob/main/images/rrt_with_obstacles.gif">
</p>