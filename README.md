# farscope_group_project
2021 cohort FARSCOPE group project

# Environment setup 

1. Install WSL2, Ubuntu 18.04, and ROS Melodic as per [this page](https://jack-kawell.com/2020/06/12/ros-wsl2/). Following the instructions to install VcXsrv is optional but recommended. 
2. Install Git (apt install git) and [GitHub CLI](https://github.com/cli/cli/blob/trunk/docs/install_linux.md) 
3. Generate a [personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) for your GitHub account and [enter it to your local git install in Ubuntu](https://docs.github.com/en/get-started/getting-started-with-git/caching-your-github-credentials-in-git) 
4. Clone the Git repo from here into your home directory (clone from ~)
5. Navigate to the ```~/farscope_group_project``` and run the following to install the UR drivers and Moveit:
      ```chmod +x fs_setup && source fs_setup```.
6. If you installed VcXsrv (and have the correct IP address set in .bashrc) you can check that everything installed correctly by running ```roslaunch ur_gazebo ur10_joint_limited.launch```, which should start a gazebo simulation of the arm.

# Starting the simulation

1. In one terminal, run ```roslaunch ur_gazebo ur10.launch```
2. In another, run ```roslaunch ur10_picking all_drivers.launch```

# Starting the robot

1. In one terminal, run ```roslaunch ur_bringup ur10_bringup.launch limited:=true robot_ip:=192.168.1.189```
2. In another, run ```roslaunch ur10_picking all_drivers.launch sim:=false```

# Making changes 

1. Send me your GitHub username so I can add you as a collaborator (means you can directly push to branches rather than everyone having to fork the repo)
2. Create a new local branch to make related changes 
3. Push local branch to remote 
4. When work on the branch is complete, open a pull request to merge the remote branch with main, which I (James) will review before merging 
5. Don't try to commit directly to remote main (it won't work) 
