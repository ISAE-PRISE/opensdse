set style function lines
set size 1.0, 1.0
set origin 0.0, 0.0
set multiplot layout 2,2 title "SDSE Data Logger plots"
set size 0.5,0.5
set origin 0.0,0.5
set grid
unset key
#
set title "Altitude evolution"
set size 0.5,0.5
set origin 0.0,0.5
set xrange [0:600]
set yrange [0:15000]
set xtics "200" , "400" , "600"
set datafile separator ","
plot filename using 1:2 title "FDM Altitude evolution" with lines
#
set title "Elevator Command"
set size 0.5,0.5
set origin 0.5,0.5
set xrange [0:600]
set yrange [-20:40]
set datafile separator ","
plot filename using 1:3 title "Right Elevator Command evolution" with lines
#
set title "Engine Command"
set size 0.5,0.5
set origin 0.0,0.0
set xrange [0:600]
set autoscale y
set datafile separator ","
plot filename using 1:5 with lines
#
pause 0.5
replot
