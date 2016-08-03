# Gnuplot graph generation script
# Gregory Kuhlmann, 2002

# Color output
set terminal postscript eps color solid "Helvetica" 24

# Black & White output
#set terminal postscript eps monochrome dashed "Helvetica" 24

# Output file
set output "./graph.eps"

# Appearance
set style data lines
set border 3
set xtics nomirror
set ytics nomirror
set multiplot

do for [i=1:15] {
    set style line i linewidth 4
}

# Axes
set xrange [0:]
set xlabel "Training Time (hours)"

set yrange [0:]
set ylabel "Episode Duration (seconds)"

# Plot Data
plot \
"20160802223622-random.out" w lp lw 3, \
"20160802223632-hand.out" w lp lw 3, \
"20160802223644-hold.out" w lp lw 3, \
"20160802223654-hive-0-Q.out" w lp lw 3, \
"20160802223704-hive-1-Q.out" w lp lw 3, \
"20160802223716-hive-2-Q.out" w lp lw 3, \
