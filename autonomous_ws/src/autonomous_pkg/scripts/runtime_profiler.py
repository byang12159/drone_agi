from picam_aruco import detect_aruco
from line_profiler import LineProfiler

def hello():
    count =0
    for i in range(1000):
        count +=1
    print(count)

lp = LineProfiler()

lp_wrapper=lp(hello)
lp_wrapper()
lp.print_stats()