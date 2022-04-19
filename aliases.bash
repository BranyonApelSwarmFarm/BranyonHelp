#!/bin/bash/sh

source /opt/ros/noetic/setup.bash

###
# awk -F "," '$1 == 296 && $9 >= 36' log

### Check robot notifiers
alias bWait='systemctl --user | grep wait-for-robot'

### Restart Zerotier
alias brz='sudo systemctl restart zerotier-one.service'

### Checkout client paddocks
alias bJeude='git checkout dev-Jeude'
alias bViridas='git checkout dev-Viradas'
alias bBendee='git checkout dev-bendee'

### Conda / QGIS shortcuts
alias bQGIS='conda activate qgis_stable;qgis'

### Mount Monash Puppet folder
alias bMon='sshfs configmaster.swarmfarm.com:/etc/puppetlabs/code/environments/branyon/ /home/branyon.apel/Monash/Puppet/ && cd ~/Monash/Puppet && code .'

### ros access client
alias bRos='source /etc/profile.d/swarmfarm-ros-access-client.sh;ros-access-client mike -f 172.20.60.6'

alias bHead='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotLocalisation/gps/fix/heading_track_over_ground /SwarmbotLocalisation/gps/fix/dual_antenna_heading /SwarmbotLocalisation/gps/fix/heading/data'

### Spray pressure
alias bSpray='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotAttachments/sprayer_v1/spray_pressure /SwarmbotAttachments/sprayer_v1/spray_pressure_reference'

### Axle Tracking
alias bSpeeds='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotPlcController/hardware_responses/wheel_linear_vel_front_left /SwarmbotPlcController/hardware_responses/wheel_linear_vel_front_right /SwarmbotPlcController/hardware_responses/wheel_linear_vel_rear_left /SwarmbotPlcController/hardware_responses/wheel_linear_vel_rear_right'

### Pressure plotting
alias bPress='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotPlcController/hardware_responses/front_mobility_pump_pressure /SwarmbotPlcController/hardware_responses/rear_mobility_pump_pressure /SwarmbotPlcController/hardware_responses/lift_tower_pressure /SwarmbotPlcController/hardware_responses/mower_pump_pressure /SwarmbotPlcController/hardware_responses/steer_pump_pressure'

### Spray tuning
alias bSpray='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotAttachments/sprayer_v1/spray_pressure /SwarmbotAttachments/sprayer_v1/spray_pressure_reference'

### Steering tuning
alias bSteer='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotPlcController/hardware_responses/articulation_angle /SwarmbotPlcController/steering_command /SwarmbotPlatformController/distance_to_path/current'

### Drive tuning
alias bDrive='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rqt_plot /SwarmbotPlcController/drive_velocity_command /SwarmbotPlcController/hardware_responses/wheel_linear_vel_average /SwarmbotLocalisation/gps/fix/velocities/ground'

### Run paddock tester
alias bpt='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rosrun job_maintainer paddock_tester /home/branyon.apel/swarmbotCommon/PaddockDefinitions'
alias bpt2='rosrun job_maintainer work_zone_simplifier 56J . definition.txt "" 0.0 0'

### Visualise point clouds
alias bVis='source ~/swarmbot4/swarmbot_ws/devel/setup.sh;roslaunch safety visualisation.launch'

### Folder shortcuts
alias bPad='cd ~/swarmbotCommon/PaddockDefinitions/'
alias bHELP='cd ~/BranyonHelp/'

### Git shortcuts
alias bWIP='git add .; git commit -m "WIP"'

### To get an auth token
# curl https://authentication.swarmfarm.com/authenticate -u branyon.apel

### To get coverage data
#curl -H "Authorization: Bearer $TOKEN" https://v-romeo.swarmfarm.com:15434/v4/jobData/coverageMap?downloaderId=e91fab47-51a5-4d41-aa50-db63b3926f42 > ~/dump.txt


check_for_ssh_add(){
	local mykey=($(cat ~/.ssh/id_ed25519.pub))
	echo "checking for '${mykey[2]}'"
	local mykey_name=${mykey[2]}
	local listkeys=($(ssh-add -l))
	local match=0
	for t in ${listkeys[@]}; do
		#echo $t
		#echo $mykey_name
		if [[ $t == $mykey_name ]]; then
			match=1
		fi
	done
	if [[ $match == 0 ]];  then
		ssh-add
	fi
}

### Bootstrapper simplifier
bbootstrap(){
	swarmfarm-bootstrapper-client -c ${1} -p ${2} --stage ${3} --environment ${4:-branyon} -u ${5:-branyon.apel}
}

### SSH simplifier
bs(){
	sshbot $(getrobotname $1)
}

copy() {

	local args=$(get_args $@)
	local nonargs=$(clear_args $@)
	
	set -- $nonargs
	
	local path=$(sfdir $2)	

	echo "rsync --progress $(getrobotstring $1 $args):$path$3 $4"
	rsync --progress $(getrobotstring $1 $args):$path$3 $4
}

getrobotname(){

	local re='^[0-9]+$'
	if [[ ${#1} < 2 ]]; then
	
		local num=""
	elif  ! [[ ${1:1:1} =~ $re ]]; then
		local num=""
	else
		local num=${1:1:1}
	fi
	
	case "${1:0:1}" in

		a|A)
			echo "alpha$num"
			;;
		b|B)
			echo "bravo$num"
			;;
		c|C)
			echo "charlie$num"
			;;
		d|D)
			echo "delta$num"
			;;
		e|E)
			echo "echo$num"
			;;
		f|F)
			echo "foxtrot$num"
			;;
		g|G)
			echo "golf$num"
			;;
		h|H)
			echo "hotel$num"
			;;
		i|I)
			echo "indigo$num"
			;;
		j|J)
			echo "juliet$num"
			;;
		k|K)
			echo "kilo$num"
			;;
		l|L)
			echo "lima$num"
			;;
		m|M)
			echo "mike$num"
			;;
		n|N)
			echo "november$num"
			;;
		o|O)
			echo "oscar$num"
			;;
		p|P)
			echo "papa$num"
			;;
		q|Q)
			echo "quebec$num"
			;;
		r|R)
			echo "romeo$num"
			;;
		s|S)
			echo "sierra$num"
			;;
		t|T)
			echo "tango$num"
			;;
		u|U)
			echo "uniform$num"
			;;
		v|V)
			echo "victor$num"
			;;
		w|W)
			echo "whiskey$num"
			;;
		x|X)
			echo "xray$num"
			;;
		y|Y)
			echo "yankee$num"
			;;
		z|Z)
			echo "zulu$num"
			;;
		*)
			echo "***invalid robot"
			;;

	esac
}
