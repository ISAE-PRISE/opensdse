set title "Simulation vs Real Time"
set autoscale x
#set autoscale y
set yrange [0:30]
set datafile separator ","
plot filename using 1:96 title "Simulation steps (Ms)" with lines, filename using 1:97 title "Real Time steps (Ms)" with lines
#
pause 0.5
replot
