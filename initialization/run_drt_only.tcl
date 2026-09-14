read_lef $::env(DRT_TECH_LEF)
read_lef $::env(DRT_CELL_LEF)
read_def $::env(DRT_INPUT_DEF)
read_guides $::env(DRT_INPUT_GUIDE)

set_thread_count [expr {int($::env(DRT_THREADS))}]

detailed_route \
  -output_drc $::env(DRT_DRC_REPORT) \
  -output_maze $::env(DRT_MAZE_LOG) \
  -verbose 0

write_def $::env(DRT_OUTPUT_DEF)