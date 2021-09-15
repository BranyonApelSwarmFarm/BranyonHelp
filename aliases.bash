#!/bin/bash/sh

### Name processing
export sf_usr="branyon.apel"

### Check robot notifiers
alias bWait='systemctl --user | grep wait-for-robot'

### Checkout client paddocks
alias bJeude='git checkout dev-Jeude'
alias bViridas='git checkout dev-Viradas'
alias bBendee='git checkout dev-bendee'

### ros access client
alias bRos='source /etc/profile.d/swarmfarm-ros-access-client.sh;ros-access-client mike -f 172.20.60.6'

### Run paddock tester
alias bpt='source ~/swarmbot4/swarmbot_ws/devel/setup.bash;rosrun job_maintainer paddock_tester /home/branyon.apel/swarmbotCommon/PaddockDefinitions'

### Folder shortcuts
alias bPad='cd ~/swarmbotCommon/PaddockDefinitions/'

### Git shortcuts
alias bWIP='git add .; git commit -m "WIP"'

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
	sshbot $(getrobotname $1) -u $sf_usr
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
