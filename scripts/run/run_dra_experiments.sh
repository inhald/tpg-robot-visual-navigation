date_code=`date +%Y_%m_%d_%H_%M_%S`_`git rev-parse --short HEAD`

##inverted_double_pendulum static
#env="mj_inverted_double_pendulum"
#exp_code="${env}_static_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
##sed -i '/time/s/0\-3:00/0\-0:30/' experiments/${exp_code}/tpg-run-slurm.sh
#cd $TPG/experiments/${exp_code}
#for s in `seq 1 10`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##inverted_double_pendulum dynamic
#env="mj_inverted_double_pendulum"
#exp_code="${env}_dynamic_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 1 10`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG

##hopper static
#env="mj_hopper"
#exp_code="${env}_static_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##hopper dynamic
#env="mj_hopper"
#exp_code="${env}_dynamic_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##half_cheetah static
#env="mj_half_cheetah"
#exp_code="${env}_static_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##half_cheetah dynamic
#env="mj_half_cheetah"
#exp_code="${env}_dynamic_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##walker2d static
#env="mj_walker2d"
#exp_code="${env}_static_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##walker_2d dynamic
#env="mj_walker2d"
#exp_code="${env}_dynamic_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##ant static
#env="mj_ant"
#exp_code="${env}_static_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
##ant dynamic
#env="mj_ant"
#exp_code="${env}_dynamic_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
## reacher static ###################################################
#env="mj_reacher"
#exp_code="${env}_static_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG
#
## reacher dynamic ##################################################
#env="mj_reacher"
#exp_code="${env}_dynamic_${date_code}"
#mkdir "experiments/${exp_code}"
#cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
#cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
#cd $TPG/experiments/${exp_code}
#for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
#cd $TPG

# 5_task static ###################################################
env="mj_5_task"
exp_code="${env}_static_${date_code}"
mkdir "experiments/${exp_code}"
cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
cd $TPG/experiments/${exp_code}
for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
cd $TPG

# 5_task dynamic ##################################################
env="mj_5_task"
exp_code="${env}_dynamic_${date_code}"
mkdir "experiments/${exp_code}"
cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
cd $TPG/experiments/${exp_code}
for s in `seq 100 110`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
cd $TPG
