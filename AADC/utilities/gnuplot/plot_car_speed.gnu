reset
set xrange [0:120]
set yrange [-2:2]
set term png
set output "car_speed.png"
set datafile separator " "
plot "car_speed.dat" using 1:2 title 'encoder' with lines, \
	"car_speed.dat" using 1:3 title 'IMU Acc Y' with lines
#pause 2
#reread
