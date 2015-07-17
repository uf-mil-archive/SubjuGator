
function sub_mission(){
echo "rosrun mission_core run_missions sub_launch.missions.$1"
rosrun mission_core run_missions sub_launch.missions.$1

}