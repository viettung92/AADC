reset
#set xrange [-5:5]
#set yrange [-5:5]
set term png
set output "point_cloud.png"
set datafile separator " "
splot "/tmp/point_cloud.dat" using 3:1:2 with lines
