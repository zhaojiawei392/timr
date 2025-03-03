# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
source /opt/ros/jazzy/setup.bash
alias buildros="cd ~/Projects/timr/ros2_ws && colcon build && source ~/Projects/timr/ros2_ws/install/setup.bash"

# Add to your ~/.bashrc
function setupcan() {
    if [ $# -ne 2 ]; then
        echo "Usage: setupcan <ttyACM number> <can number>"
        echo "Example: setupcan 0 0 (for /dev/ttyACM0 and can0)"
        echo "Example: setupcan 1 1 (for /dev/ttyACM1 and can1)"
        return 1
    fi

    local device="/dev/ttyACM$1"
    local caninterface="can$2"

    if [ ! -e "$device" ]; then
        echo "Error: $device does not exist"
        return 1
    fi

    echo "Setting up $caninterface using $device..."
    sudo slcand -o -c -s8 "$device" "$caninterface" &&
    sudo ip link set "$caninterface" down &&
    sudo ip link set "$caninterface" type can bitrate 1000000 &&
    sudo ip link set "$caninterface" up &&
    echo "CAN setup complete on $caninterface using $device"
}

function pubjoint() {
    if [ $# -ne 1 ] && [ $# -ne 6 ]; then
        echo "Usage: pubjoints <position>  # Same position for all joints"
        echo "   or: pubjoints <j1> <j2> <j3> <j4> <j5> <j6>  # Individual positions"
        echo "Example: pubjoints 0.1"
        echo "     or: pubjoints 0.1 0.2 0.3 0.4 0.5 0.6"
        return 1
    fi

    local j1 j2 j3 j4 j5 j6
    
    if [ $# -eq 1 ]; then
        # If one argument, use it for all joints
        j1=$1; j2=$1; j3=$1; j4=$1; j5=$1; j6=$1
    else
        # If six arguments, use individual values
        j1=$1; j2=$2; j3=$3; j4=$4; j5=$5; j6=$6
    fi

    ros2 topic pub -r 250 /target_joint_state sensor_msgs/msg/JointState "{
        header: {
            stamp: {sec: 0},
            frame_id: 'base_link'
        },
        name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        position: [$j1, $j2, $j3, $j4, $j5, $j6],
        velocity: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }"
}

deletecan() {
    if [ -z "$1" ]; then
        echo "Usage: deletecan <number>"
        echo "Example: deletecan 0 (to delete can0)"
        return 1
    fi

    local can_interface="can$1"

    # Check if the interface exists
    if ip link show "$can_interface" &> /dev/null; then
        echo "Removing $can_interface..."
        
        # Bring down the interface
        sudo ip link set "$can_interface" down
        
        # Delete the interface
        sudo ip link delete "$can_interface"
        
        echo "$can_interface has been removed"
    else
        echo "Interface $can_interface not found"
        return 1
    fi
}
