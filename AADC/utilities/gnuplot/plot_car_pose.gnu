reset
set xrange [-5:5]
set yrange [-5:5]
set term png
set output "car_pose.png"
set datafile separator " "
x_last = "`tail -1 car_pose.dat | awk '{print $1}'`"
y_last = "`tail -1 car_pose.dat | awk '{print $2}'`"
yaw_last = "`tail -1 car_pose.dat | awk '{print $3}'`"

x_arrow = x_last + 1.5 * cos(yaw_last)
y_arrow = y_last + 1.5 * sin(yaw_last)

set arrow from x_last,y_last to x_arrow, y_arrow
plot "car_pose.dat" using 1:2 with lines
pause 2
reread
