# Robot Visual Navigation using TPG

This code reproduces results from the paper:

Neelkant Teeluckdharry, Stephen Kelly. Robot Visual Navigation using Tangled Program Graphs





We can rapidly evolve visual navigation policies on a high-performance computing cluster such as those on the Digital Research Alliance of Canada. 

## Pre-requisites
Load the apptainer module `module load apptainer` and set `$TPG` to the environment variable for this directory.

## Build the container
First `cd apptainer`, then complete the build the container by running `apptainer build gazebo_tpg.sif gazebo_tpg.def`. 

## Build the TPGExperiment binary file
Within the same directory, run `sbatch build_and_test.sbatch`. These commands are useful for monitoring the job: `watch -n 5 squeue -u $USER` and `tail -f slurm<your_job_number>.out`.

Ensure that all testcases pass.


## Running a job
Copy the config file to a new directory in experiments

`mkdir -p $TPG/experiments/gazebo_turtlebot4 && cp $TPG/configs/gazebo_turtlebot4.yaml $TPG/experiments/gazebo_turtlebot4`

Then, run the slurm script from within the experiment directory. 

`sbatch ../../scripts/run/tpg-gazebo.sh -s <your_seed_number> -p ./${PWD##*/}.yaml`



