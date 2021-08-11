#!/bin/bash/sh

### Name processing
export username="branyon.apel"

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

getrobotstring()
{
	#Defaults
	local xenial=0
	local vpn="v-"	#Change to "" to have vpn off by default

	if [ -z $1 ]; then
	
		echo "No robot specified!"
		return
	elif ! ([[ $1 == '-h' ]] || [[ $1 == '--help' ]] || [[ $1 == '--forward-help' ]] || [[ $1 == '--forward-ip-help' ]]); then
	
		local robot=$1
		shift
	fi
	POSITIONAL=()
	while [[ $# -gt 0 ]]
	do
	key="$1"
	case $key in
		
		-x|--xenial)
			local xenial=1
			shift
			;;
		-f|--focal)
			local xenial=0
			shift
			;;
		-i|--ip_only)
			local ip=1
			shift
			;;
		-v|--vpn)
			local vpn="v-"
			shift
			;;
		-l|--local)
			local vpn=""
			shift
			;;
		-r|--ros)
			local ros="-ros"
			shift
			;;
		-h|--help)
			echo "\
-h | --help		: show this message
-x | --xenial 		: forces to login as swarmbot@ 
-f | --focal		: forces to login as <username@> 
-i | --ip_only		: returns only the address of the robot not anything@
-v | --vpn		: forces \"v-\" in front of the address 
-l | --local		: forces the address to be the local address 
-r | --ros		: forces the address to have \"-ros\" on the end of it
--forward-help		: returns the help message with this line and the top line removed"
			return
			shift
			;;
		--forward-help)
			echo "\
-x | --xenial 		: forces to login as swarmbot@ 
-f | --focal		: forces to login as <username@> 
-i | --ip_only		: returns only the address of the robot not anything@
-v | --vpn		: forces \"v-\" in front of the address 
-l | --local		: forces the address to be the local address 
-r | --ros		: forces the address to have \"-ros\" on the end of it"
			return
			shift
			;;
		--forward-ip-help)
			echo "\
-v | --vpn		: forces \"v-\" in front of the address 
-l | --local		: forces the address to be the local address 
-r | --ros		: forces the address to have \"-ros\" on the end of it"
			return
			shift
			;;	
		*)
			shift
			;;
	esac
	done
	set -- "${POSITIONAL[@]}" # restore positional parameters
	
	if ! [ -z $vpn ] && ! [ -z $ros ]; then
		echo "*** You can't have both 'vpn' and 'ros' set. NOTE: vpn could be on by default, use \"-l\" to do the local address"
		return
	fi
	
	if [ -z $ip ]; then
	
		if ! [[ $xenial == 1 ]]; then
			echo "$username@$vpn$(getrobotname $robot)$ros.swarmfarm.com"
		else
			echo "swarmbot@$vpn$(getrobotname $robot)$ros.swarmfarm.com"
		fi
	else
	
		echo "$vpn$(getrobotname $robot)$ros.swarmfarm.com"
	fi
}

getprogram() {

	case "$1" in
	
		zt | zerotier | vpn | v)
			echo "zerotier-one"
			;;
		nm | network | network-manager)
			echo "network-manager"
			;;
		*)
			echo $1
			;;
	esac
}

### Argument passing

get_args()
{
	local args=""
	while [[ $# -gt 0 ]]
	do
	key="$1"
	if [[ ${key:0:1} == "-" ]]; then
		local args="$args $key"
	fi
	shift
	done
	echo $args
}

clear_args()
{
	local nonargs=""
	while [[ $# -gt 0 ]]
	do
	key="$1"
	if ! [[ ${key:0:1} == "-" ]]; then
		local nonargs="$nonargs $key"
	fi
	shift
	done
	echo $nonargs
}

split() {

	echo $(cut -d' ' -f $2 <<<"$1")

}

dec2ascii() {

	echo $(printf "\x$(printf %x $1)")	
}

### Robot Access

sfssh()
{

	local pre=$@
	while [[ $# -gt 0 ]]
	do
	key="$1"
	case $key in
		
		-p|--port-forward)
			local pf="-D 1081 "
			shift
			;;
		-h|--help)
			echo "\
-h | --help		: show this message
-p | --port-forward	: will add \"-D 1081\" to add port forwarding
$(getrobotstring --forward-help)"
			return
			;;
		*)
			shift
			;;
	esac
	done

	echo "ssh $pf$(getrobotstring $pre)"
	ssh $pf$(getrobotstring $pre)
}

sfany()
{

	local command=$1
	if ! ( [[ $command == '-h' ]] || [[ $command == '--help' ]] ); then
		shift
	else 
		local help=1
	fi
	
	local pre=$@
	POSITIONAL=()
	while [[ $# -gt 0 ]]
	do
	key="$1"
	case $key in
		-i|--ip_only)
			local ip_only=1
			shift
			;;
		-h|--help)
			local help=1
			shift
			;;
		*)
			shift
			;;
	esac
	done
	set -- "${POSITIONAL[@]}" # restore positional parameters
	
	if ! [ -z $help ]; then
	
		if [ -z $ip_only ]; then
		
			echo "$(getrobotstring --forward-help)"
		else
		
			echo "$(getrobotstring --forward-ip-help)"
		fi
		return
	fi
	
	local resp=$(getrobotstring $pre)
	
	if [[ ${resp:0:1} == '*' ]]; then
		echo "$resp"
		return
	fi
	
	echo "$command $(getrobotstring $pre)"
	$command $(getrobotstring $pre)
}

sfping()
{
	sfany ping $@ -i
}

sflookup()
{
	sfany nslookup $@ -i
}

sfros()
{
	ros-access-client $(getrobotname $1)
}

pingbot() {
	echo "Waiting for $(getrobotstring $@ -i)"
	
	until ping -c 1 $(getrobotstring $@ -i) > /dev/null; do
		sleep 1
	done
	spd-say "$(getrobotname $1) is running"
	zenity --info --text "$(date);$(getrobotname $1) is running"
	slacksend -c robot-alerts -m "$(getrobotname $1) is on"
}

pingbotreverse() {
	echo "Waiting for $(getrobotstring $@ -i)"
	
	while ping -c 1 $(getrobotstring $@ -i) > /dev/null; do
		sleep 1
	done
	spd-say "$(getrobotname $1) is dead"
	zenity --info --text "$(date);$(getrobotname $1) is dead"
}

sfscan() {

	# If in scan - we attempt to find everything on the 192.168.50.32/28 subnet range
    hosts=$(fping -a -g -q 192.168.50.32/28)
    if [ -n "$debug" ]; then
        echo "fping output"
        echo "$hosts"
    fi

    if [ -z "$hosts" ]; then
        >&2 echo "Couldn't find any hosts in the range 192.168.50.32/28"
        >&2 echo "Are you sure you are on a robot wifi and your network is configured correctly?"
    elif [ $(echo "$hosts" | wc -l) == "1" ]; then
        echo "Found one host: $hosts"
        ssh_host="$hosts"
    else 
        echo "Found multiple hosts in the range 192.168.50.32/28"
        select d in $hosts; do
            ssh_host="$d"
            break
        done
    fi
}

sfvscan() {

	for (( n=105; n<=122; n++ ))
	do
		local l=$(dec2ascii $n)
		check=$(fping -a -q $(getrobotstring $l -i -v))

		if [ -z "$check" ]; then
			echo "$(getrobotname $l) is uncontactable"
		else
			echo "$(getrobotname $l) is on"
		fi
		
		
	done

}

sfcd() {

	echo "cd $(sfdir $1)"
	cd $(sfdir $1)
}

# Copying shortcuts

rcopy() {

	local args=$(get_args $@)
	local nonargs=$(clear_args $@)
	
	set -- $nonargs
	
	local path=$(sfdir $2)	

	echo "rsync --progress $3 $(getrobotstring $1 $args):$path"
	rsync --progress $3 $(getrobotstring $1 $args):$path
}

copy() {

	local args=$(get_args $@)
	local nonargs=$(clear_args $@)
	
	set -- $nonargs
	
	local path=$(sfdir $2)	

	echo "rsync --progress $(getrobotstring $1 $args):$path$3 $4"
	rsync --progress $(getrobotstring $1 $args):$path$3 $4
}

alias slimsim="ssh alex.lawrie@slim.swarmfarm.com"

### APT

build_docker() {

	sudo docker build $2 -f build.dockerfile -t $1 ./
	export last_docker=$1
}

push_last() {

	echo "executing: \"push-packages -i $last_docker:latest\"" 
	push-packages -i $last_docker:latest	
}

check_update() {

	if [ -z $2 ]; then
		time=10
	else
		time=$2
	fi

	echo "Will check verison of $1 every $time seconds"

	while true; do
		sudo apt update
		apt policy $1
		sleep $time
	done

}

alias up='sudo apt update'

### Mapping 
reverse() {
	filename=$(cut -f 1 -d '.' <<< "$1")
	filename+="_reversed.txt"

	echo "Writing to: $filename"
	tac "$1" >> $filename
}

### Network Aliases
alias rvpn='sudo systemctl restart zerotier-one; echo "Restarted!"'
alias g="ping 8.8.8.8"

### Git Stuff
branch () {
	git branch -a | grep $1
}

### Robot plotting
alias plot_steer='rqt_plot /SwarmbotPlcController/hardware_responses/articulation_angle /SwarmbotPlcController/steering_command /distance_to_path'
alias plot_steer_and_dist='rqt_plot /SwarmbotPlcController/hardware_responses/articulation_angle /SwarmbotPlcController/articulation_angle_command /distance_to_path'
alias plot_steer_man='rqt_plot /SwarmbotPlcController/hardware_responses/articulation_angle /SwarmbotPlcController/articulation_angle_command'
alias plot_drive="rqt_plot /SwarmbotPlcController/hardware_responses/wheel_linear_vel_average /SwarmbotPlcController/drive_velocity_command /SwarmbotLocalisation/gps/velocity/velocity_ground"
alias plot_encoders="rqt_plot /SwarmbotPlcController/hardware_responses/wheel_linear_vel_front_left /SwarmbotPlcController/hardware_responses/wheel_linear_vel_front_right /SwarmbotPlcController/hardware_responses/wheel_linear_vel_rear_left /SwarmbotPlcController/hardware_responses/wheel_linear_vel_rear_right /SwarmbotLocalisation/gps/velocity/velocity_ground"
alias plot_drive_with_encoders="rqt_plot /SwarmbotPlcController/drive_velocity_command /SwarmbotLocalisation/gps/velocity/velocity_ground /SwarmbotPlcController/hardware_responses/wheel_linear_vel_front_left /SwarmbotPlcController/hardware_responses/wheel_linear_vel_front_right /SwarmbotPlcController/hardware_responses/wheel_linear_vel_rear_left /SwarmbotPlcController/hardware_responses/wheel_linear_vel_rear_right"
alias plot_spray_pressure='rqt_plot /SwarmbotAttachments/sprayer_v1/spray_pressure /SwarmbotAttachments/sprayer_v1/spray_pressure_reference'
alias sim="rviz -d swarmbot4/swarmbot_ws/src/simulation/launch/sim.rviz"
alias points="roslaunch perception visualisation.launch"
alias deps="cd ~/swarmbot4/swarmbot_ws/src/imports; sudo mk-build-deps -i; cd -"
alias monash="ssh alex.lawrie@monash.swarmfarm.com"
alias mount_puppet="sshfs alex.lawrie@monash.swarmfarm.com:/etc/puppetlabs/code/environments/lawrie/ /home/alex/puppet"
alias ssh_link='if ! grep -Fxq "ControlMaster auto" ~/.ssh/config; then echo "ControlMaster auto" >> ~/.ssh/config; fi; if ! grep -Fxq "ControlPath ~/.ssh/%r@%h:%p" ~/.ssh/config; then echo "ControlPath ~/.ssh/%r@%h:%p" >> ~/.ssh/config; fi;'
alias cmn='cd ~/swarmbotCommon'
alias cws='cd ~/swarmbot4/swarmbot_ws/'
alias sce='source ~/.bashrc'
alias sys='cd ~/SystemUtilities'


## AWS
awscopy() {

	aws s3 cp s3://swarmfarm-certificates/alex.lawrie.google_authenticator .
}

awslist() {

	aws s3
}

### Unsupported

ref_token() {

	export token=$(echo $(curl https://authentication.swarmfarm.com/authenticate -u alex.lawrie | jq .access -r))

	echo $token
}

iface_post() {

	echo "curl https://$(getrobot_new $1 0):15434/v4/jobData/$2 -H "Authorization: Bearer $token" -X POST -d \"{\"uuid\": \"7d9f499e-df79-46c2-9b01-1cc6b793448a\", \"value\": $3 }\" | jq"

	curl https://$(getrobot_new $1 0):15434/v4/jobData/$2 -H "Authorization: Bearer $token" -X POST -d "{\"uuid\": \"7d9f499e-df79-46c2-9b01-1cc6b793448a\", \"value\": $3 }" | jq

}

iface_get() {

	if [ -z $3 ]; then
		refid=""
	else
		refid="/$3"
	fi

	echo "curl https://$(getrobot_new $1 0):15434/v4/jobData/$2$refid -H "Authorization: Bearer $token" | jq"


	curl https://$(getrobot_new $1 0):15434/v4/jobData/$2$refid -H "Authorization: Bearer $token"

}

iface_delete() {

	if [ -z $3 ]; then
		refid=""
	else
		refid="/$3"
	fi

	echo "curl https://$(getrobot_new $1 0):15434/v4/jobData/$2$refid -H "Authorization: Bearer $token" -X DELETE | jq"


	curl https://$(getrobot_new $1 0):15434/v4/jobData/$2$refid -H "Authorization: Bearer $token" -X DELETE | jq

}

strip_all()
{
	count=0
	for file in $1*.bag;
	do
		echo "Filtering $1${file##*/} into $count.bag"
		rosrun swarmbot bag_stripper_node $1${file##*/} $count.bag
		((count = count + 1))
	done
}

check_all()
{
	for file in $1*.bag;
	do
		echo "Checking $1${file##*/}"
		rosbag info $1${file##*/} | grep size
	done
}

sfnuvo()
{
	local user=root

	POSITIONAL=()
	while [[ $# -gt 0 ]]
	do
	key="$1"
	case $key in
		
		-s|--start)
			local start=1
			shift
			;;
		-k|--kill|--poweroff)
			local kill=1
			shift
			;;
		-r|--reboot)
			local reboot=1
			shift
			;;
		-i|--info|--status)
			local status=1
			shift
			;;
		-l|--login)
			local login=1
			shift
			;;
		-u|--user)
			shift
			local user=$1
			shift
			;;
		-m|--machine)
			shift
			local machine=$1
			shift
			;;
		--slave | --sm)
			shift
			local slave=1
			;;
		--master | --mm)
			shift
			local master=1
			;;
		-b | --bm | --both)
			shift
			local slave=1
			local master=1
			;;
		-h|--help)
			echo "\
-h | --help			: show this message
-s | --start 			: start the machine
-k | --kill | --poweroff	: turns the machine off
-r | --reboot			: restarts the machine
-i | --info | --status		: shows the status of the machine
-l | --login			: logs in to the machine
-m | --machine			: chose which machine (slave, master or both (s, m or b))
--slave | --sm			: select the slave
--master | --mm 		: select the master
-b | --both			: select both"		
			return
			shift
			;;
		*)
			echo "$1 is not a valid argument"
			return
			shift
			;;
	esac
	done
	set -- "${POSITIONAL[@]}" # restore positional parameters
	
	case $user in
	
		"a")
			local user=alex.lawrie
			;;
		"r")
			local user=root
			;;
	esac
	
	local sum=$((start + kill + reboot + status + login ))
	
	if [[ $sum > 1 ]]; then
	
		echo "You can only chose one of start, kill, reboot, info or login"
		return
	elif [[ $sum < 1 ]]; then
		echo "You must chose one of start, kill, reboot, info or login"
		return 
	fi
	
	case $machine in
	
		s|slave)
			local slave=1
			;;
		m|master)
			local master=1
			;;
		b|both)
			local slave=1
			local master=1
			;;
	esac
	
	if ! [ -z $start ]; then
		local command="start "
	elif ! [ -z $kill ]; then
		local command="poweroff "
	elif ! [ -z $reboot ]; then
		local command="reboot "
	elif ! [ -z $login ]; then
		if [[ $slave == 1 ]] && [[ $master == 1 ]]; then
			echo "You can't login to both machines!"
			return
		fi
		local command="shell $user@"
	elif ! [ -z $status ]; then
		local command="status "
	fi
	
	if [[ $slave == 1 ]]; then
		echo "sudo machinectl $command""focal-nuvo-slave"
		sudo machinectl $command"focal-nuvo-slave"
	fi
	if [[ $master == 1 ]]; then
		echo "sudo machinectl $command""focal-nuvo-master"
		sudo machinectl $command"focal-nuvo-master"
	fi
	
}

