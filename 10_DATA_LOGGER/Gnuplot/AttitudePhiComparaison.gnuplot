set title "Real vs Measured Aircraft Phi angle"
set autoscale x
set autoscale y
set datafile separator ","
plot filename using 1:36 title "Commanded" with lines, filename using 1:62 title "Effective" with lines
