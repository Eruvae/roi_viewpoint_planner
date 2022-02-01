if [ $# -eq 0 ]
  then
    echo "No arguments supplied"
    exit
fi
resolutions=(0.005 0.01 0.02)
wrl="${1}.wrl"
bv="${1}.binvox"
for i in "${resolutions[@]}"
do
  grid=$(echo "2/$i" | bc)
  bv_res="${1}_${i}.binvox"
  bt_res="${1}_${i}.bt"
  ./binvox -e -bb -1 -1 -1 1 1 1 -d $grid -rotx $wrl
  mv $bv $bv_res
  binvox2bt $bv_res -o $bt_res
  rm $bv_res
done

