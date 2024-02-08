for i in $(seq 0 8);	do
	echo $i
	rosservice call /ur5/vacuum_gripper$i/on &
done

wait