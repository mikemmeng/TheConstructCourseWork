#! /usr/bin/env python

####################################
#Claude roadmap output.
####################################
claude_roadmap = """

ROS/ROS2 Learning Roadmap for Job-Ready Skills (6-12 Months)
Month 1: Foundations

Linux Basics (1-2 weeks)

Install Ubuntu (recommended for ROS)
Learn basic terminal commands
Understand file system and permissions
Practice with bash scripting


Programming Basics (2-3 weeks)

Python fundamentals (variables, loops, functions)
C++ basics if time permits (important for performance-critical applications)
Object-oriented programming concepts
Version control with Git



Month 2: ROS Core Concepts

ROS Installation & Setup (1 week)

Install ROS2 (recommended as it's the future) or ROS1
Configure environment
Understand workspace structure


ROS Fundamentals (3 weeks)

Nodes, topics, services, actions
Publishers and subscribers
ROS parameter server
Launch files
Complete basic tutorials from ROS wiki



Month 3: Intermediate ROS Skills

Robot Modeling (2 weeks)

URDF (Unified Robot Description Format)
Xacro for modular URDF
Gazebo simulation basics


Visualization and Debugging (2 weeks)

RViz for visualization
rqt tools for debugging
rosbag for data recording/playback



Month 4: Perception & Navigation

Sensors and Perception (2 weeks)

Working with common sensors (cameras, LiDAR, IMUs)
Point cloud processing with PCL
Computer vision basics with OpenCV


Navigation Stack (2 weeks)

Map creation (SLAM)
Localization techniques
Path planning algorithms
Nav2 (for ROS2)



Month 5: Manipulation & Real Projects

Robot Manipulation (2 weeks)

Forward and inverse kinematics
MoveIt! framework
Motion planning


First Real Project (2 weeks)

Build a complete application (e.g., autonomous navigation or pick-and-place)
Integrate multiple ROS components
Document your process and results for your portfolio



Month 6: Advanced Topics & Industry Focus

Real-Time Systems (1 week)

Real-time OS concepts
Deterministic behavior
Performance optimization


Industry-Specific Skills (3 weeks)

Choose a focus area (see below) and build deeper expertise
Network with professionals in your chosen area
Attend local ROS meetups or online events



Industry Focus Areas (choose one):

Manufacturing Automation: Pick and place, assembly tasks
Warehouse/Logistics: Fleet management, path optimization
Service Robotics: Human-robot interaction, safety systems
Autonomous Vehicles: Sensor fusion, behavior planning
Healthcare: Precision control, safety-critical systems

Months 7-9: Project Portfolio Development

Build 2-3 Substantial Projects (3 months)

Each project should demonstrate different skills
Focus on industry-relevant applications
Document thoroughly (code, videos, presentations)
Use Git for version control and showcase on GitHub



Months 10-12: Professional Development

Code Quality & Best Practices (2 weeks)

Learn continuous integration for robotics
Unit testing in ROS
Code documentation
Performance profiling


Certification & Job Preparation (2 weeks)

Consider ROS certification if available
Prepare technical interview answers
Create demo videos of your projects


Job Search Strategy (2 months)

Refine resume highlighting ROS skills
Network with robotics companies
Contribute to open-source ROS packages
Prepare for technical interviews with robotics-specific questions



Learning Resources

Official Resources

ROS Wiki and Tutorials
ROS Discourse Forum
ROS Answers (Q&A)


Online Courses

The Construct (ROS online courses)
Coursera/Udemy ROS courses
edX robotics courses


Books

"Programming Robots with ROS" by Morgan Quigley
"Mastering ROS for Robotics Programming" by Lentin Joseph
"Robot Operating System (ROS) for Absolute Beginners" by Lentin Joseph


Communities

ROS Discourse
GitHub repositories
LinkedIn ROS groups
Local robotics meetups



Tips for Success

Consistent Practice: Spend at least 15-20 hours weekly
Build Real Things: Always apply concepts to actual projects
Document Everything: Create a blog or detailed GitHub repositories
Connect with Community: Join ROS forums and robotics groups
Focus on Industry Needs: Look at job postings to identify in-demand skills

"""

print(claude_roadmap)
