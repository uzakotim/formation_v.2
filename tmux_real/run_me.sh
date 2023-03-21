#!/bin/bash
gnome-terminal -e 'bash -c "sshpass -p 'mrs' ssh mrs@192.168.69.102; cd ~; cd ~/timur_workspace/src/formation_v.2/tmux_real; ./tmux01.sh; bash"' &
gnome-terminal -e 'bash -c "sshpass -p 'mrs' ssh mrs@192.168.69.144; cd ~; cd ~/timur_workspace/src/formation_v.2/tmux_real; ./tmux02.sh; bash"' &
gnome-terminal -e 'bash -c "sshpass -p 'mrs' ssh mrs@192.168.69.161; cd ~; cd ~/timur_workspace/src/formation_v.2/tmux_real; ./tmux03.sh; bash"'
