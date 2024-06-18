from picam_aruco import detect_aruco
from line_profiler import LineProfiler

lp = LineProfiler()

lp_wrapper=lp(detect_aruco)
lp_wrapper()
lp.print_stats()