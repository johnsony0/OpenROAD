read_lef $::env(DRT_TECH_LEF)
read_lef $::env(DRT_CELL_LEF)
read_def $::env(DRT_INPUT_DEF)
read_guides $::env(DRT_INPUT_GUIDE)

set_thread_count [expr {int($::env(DRT_THREADS))}]

# Optional DRT_END_ITER stops detailed routing after that DR iteration.
set end_iter_args {}
if {[info exists ::env(DRT_END_ITER)] && $::env(DRT_END_ITER) ne ""} {
  set end_iter_args [list -droute_end_iter $::env(DRT_END_ITER)]
}

detailed_route \
  -output_drc $::env(DRT_DRC_REPORT) \
  -output_maze $::env(DRT_MAZE_LOG) \
  -verbose [expr {[info exists ::env(DRT_VERBOSE)] ? $::env(DRT_VERBOSE) : 0}] \
  {*}$end_iter_args

write_def $::env(DRT_OUTPUT_DEF)