#!/bin/bash
#SBATCH --account=def-skelly
#SBATCH --job-name=tpg_build
#SBATCH --time=00:30:00
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G

module load apptainer
TPG=$HOME/projects/def-skelly/teeluckn/tpg
IMG=$TPG/apptainer/gazebo_tpg.sif
MUJOCO_HOST=$HOME/projects/def-skelly/teeluckn/mujoco-3.2.2
INST=sim_$SLURM_JOB_ID

# start isolated container on this compute node
apptainer instance start \
	--env IGN_PARTITION=job_${SLURM_JOB_ID} \
	--writable-tmpfs \
	-B $TPG \
	-B $MUJOCO_HOST \
	$IMG $INST

# compile and run tests inside the instance
apptainer exec instance://$INST bash -e <<'INSIDE'
  export TPG=$HOME/projects/def-skelly/teeluckn/tpg
  export IGN_PARTITION="job_${SLURM_JOB_ID}"
  export IGN_TRANSPORT_TOPIC_STATISTICS=0
  export IGN_IPC_LOCK_DISABLE=1
  export LIBGL_ALWAYS_SOFTWARE=1
  export MUJOCO=$HOME/projects/def-skelly/teeluckn/mujoco-3.2.2
  cd $TPG && make optimized
INSIDE

# stop the instance
apptainer instance stop $INST
echo "Finished Tests!"
