# PACNav
This repository contains the scripts and code to run the Gazebo simulations and real-world experiments for the paper titled "PACNav: A Collective Navigation Approach for UAV Swarms Deprived of Communication and External Localization".

### Step 1) Installation and building containers.
- Install `docker` and `singularity` using the install scripts provided in `pacnav/install` directory. You may skip this if you already have these dependencies installed.
- Build the singularity image by running `./singularity/recipe/build.sh`.

### Step 2) Running the container and building the ROS packages.
- After a successful build, run the singularity container using `./singularity/run_singularity.sh` script.
- The script will mount the `singularity/user_ros_workspace` directory into the singularity container.
- Build the required ROS packages by running `catkin build -c` inside `singularity/user_ros_workspace`.

### Step 3) Running the mutli-UAV simulation.
- After a successful `catkin build`, run the multi-UAV simulation using ```./singularity/user_ros_workspace/src/mrs_swarm_core/simulation/simulate_swarm.sh -f config/sim_config.yaml```
- The parameters for setting up the multi-UAV simulation are in the `singularity/user_ros_workspace/src/mrs_swarm_core/simulation/config/sim_config.yaml` file. You can play around with them as you please.

### Citation
Please use the citation below if you find our work useful :blush:.

```
@ARTICLE{Ahmad2022Bioinspired,
  title = {{PACNav: A collective navigation approach for UAV swarms deprived of communication and external localization}},
  author = {{Ahmad}, Afzal and {Bonilla Licea}, Daniel and {Silano}, Giuseppe and {Baca}, Tomas and {Saska}, Martin},
  doi = {10.1088/1748-3190/ac98e6},
  group = {journals},
  journal = {Bioinspiration & Biomimetics},
  year = {2022},
  organization = {IOP Science},
  month = oct,
  preprint = {http://mrs.felk.cvut.cz/data/papers/Bioinspired_Afzal.pdf},
  code = {https://github.com/ctu-mrs/pacnav},
  link = {https://iopscience.iop.org/article/10.1088/1748-3190/ac98e6}
}
```

YouTube video
--------------

In this section a video showcasing the validity and the effectviness of the approach is reported. Further videos can be found in the related publication. 

[![PACNav: A collective navigation approach for UAV swarms deprived of communication and external localization](https://github.com/ctu-mrs/pacnav/wiki/img/img_bio_22.png)](https://youtu.be/Cpuqx7Imrz4 "PACNav: A collective navigation approach for UAV swarms deprived of communication and external localization")
