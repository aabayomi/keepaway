# Gnuplot histogram generation script
# Gregory Kuhlmann, 2002

# Color output
set terminal postscript eps color solid "Helvetica" 24

# Black & White output
#set terminal postscript eps monochrome dashed "Helvetica" 24

# Output file
set output "./hist.eps"

# Appearance
set border 3
set xtics nomirror
set ytics nomirror
set multiplot

set style fill transparent solid 0.5 #noborder

# Axes
set xrange [0:]
set xlabel "Episode Duration (seconds)"

set yrange [0:]
set ylabel "Occurences"

# Plot Data
plot \
"20160802223622-random.hist" w boxes, \
"20160802223632-hand.hist" w boxes, \
"20160802223644-hold.hist" w boxes, \
"20160802223654-hive-0-Q.hist" w boxes, \
"20160802223704-hive-1-Q.hist" w boxes, \
"20160802223716-hive-2-Q.hist" w boxes, \
