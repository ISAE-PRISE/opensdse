set title "Commanded vs Effective Elevator Deflection"
set autoscale x
set autoscale y
set datafile separator ","
plot filename using 1:13 title "Commanded" with lines, filename using 1:24 title "Effective" with lines
