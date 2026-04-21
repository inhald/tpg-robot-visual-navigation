#!/bin/bash
#SBATCH --account=def-skelly
#SBATCH --job-name=tpg_gazebo
#SBATCH --nodes=1
#SBATCH --ntasks=26
#SBATCH --time=12:00:00
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=6G

module load apptainer openmpi
export TPG=$HOME/projects/def-skelly/teeluckn/tpg
export IMG=$TPG/apptainer/gazebo_tpg.sif

mkdir -p checkpoints
mkdir -p logs

export PMIX_MCA_psec=^munge
# export UCX_SHM_DEVICES=posix,sysv
unset UCX_SHM_DEVICES
export UCX_TLS=shm,tcp,self

#defaults
mode=0 #Train:0, Replay:1, Debug:2
seed_tpg=42
parameters_file="parameters_TPG.yaml"

while getopts m:p:s: flag
do
   case "${flag}" in
      m) mode=${OPTARG};;
      p) parameters_file=${OPTARG};;
      s) seed_tpg=${OPTARG};;
   esac
done

cat > run_tpg.sh << 'EOF'
#!/usr/bin/env bash
export IGN_PARTITION="job_${SLURM_JOB_ID}_${OMPI_COMM_WORLD_RANK}"
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$TPG/datasets/gazebo_models/models 
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$TPG/build/datasets/gazebo_models/plugins 
exec $TPG/build/release/experiments/TPGExperimentMPI \
  parameters_file="$1" \
  seed_tpg="$2" \

EOF
chmod +x run_tpg.sh







mpirun -np "$SLURM_NTASKS" \
        --map-by ppr:$SLURM_NTASKS:node \
        apptainer exec \
                --bind "${TPG}:${TPG}" \
                --env IGN_TRANSPORT_TOPIC_STATISTICS=0 \
                --env IGN_IPC_LOCK_DISABLE=1 \
                --env LIBGL_ALWAYS_SOFTWARE=1 \
                "$IMG" \
		./run_tpg.sh "${parameters_file}" "${seed_tpg}" \
		1> logs/tpg.${seed_tpg}.$$.std \ 
		2> logs/tpg.${seed_tpg}.$$.err
