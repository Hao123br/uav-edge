set view map;
set xlabel "X"
set ylabel "Y"
set xrange [0:2500]
set yrange [2500:0]
set cblabel "SINR (dB)"
set term png size 1280, 960
set output "rem-start.png"
unset key
plot "rem-start.out" using ($1):($2):(10*log10($4)) with image
