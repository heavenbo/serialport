#!/bin/bash
i=$1
if [ $i -eq 1 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/pack.mp3
elif [ $i -eq 2 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/blue_cir.mp3
elif [ $i -eq 3 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/red_cir.mp3
elif [ $i -eq 4 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/blue_square.mp3
elif [ $i -eq 5 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/red_square.mp3
elif [ $i -eq 6 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/blue_tri.mp3
elif [ $i -eq 7 ]
then
ffplay -autoexit /home/boyheaven/Vscode/work/Mplayer/mp3/red_tri.mp3
fi
