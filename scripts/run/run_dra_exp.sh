date_code=`date +%Y_%m_%d_%H_%M_%S`_`git rev-parse --short HEAD`

#ant static
env="mj_ant"
exp_code="${env}_static_${date_code}"
mkdir "experiments/${exp_code}"
cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
sed -i '/p_memory_size/s/p_memory_size:\ 1.0/p_memory_size:\ 0.0/' experiments/${exp_code}/${exp_code}.yaml
cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
cd $TPG/experiments/${exp_code}
for s in `seq 1 10`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
cd $TPG

#ant dynamic
env="mj_ant"
exp_code="${env}_dynamic_${date_code}"
mkdir "experiments/${exp_code}"
cp configs/${env}.yaml experiments/${exp_code}/${exp_code}.yaml
cp $TPG/scripts/run/tpg-run-slurm.sh experiments/${exp_code}
cd $TPG/experiments/${exp_code}
for s in `seq 1 10`; do sbatch ./tpg-run-slurm.sh -s $s -p ./${PWD##*/}.yaml; done
cd $TPG

