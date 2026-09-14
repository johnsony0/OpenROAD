# ~/.local/bin/bazel build //:openroad
# ./bazel-bin/openroad -no_init -exit initialization/run_gcd.tcl
# env DRT_DUMP_GG_DIR="top_gcd/ggdump"
# env DRT_DUMP_EXP_DIR="top_gcd/expdump"
# env DRT_DUMP_COST_DIR="top_gcd/costdump"
# env DRT_DUMP_MIN_ITER=1   
# env DRT_DUMP_ITER=1

read_lef "test/Nangate45/Nangate45_tech.lef"
read_lef "test/Nangate45/Nangate45_stdcell.lef"
read_def "src/drt/test/gcd_nangate45_preroute.def"
read_guides "src/drt/test/gcd_nangate45.route_guide"

set_thread_count 1

detailed_route -output_drc initialization/reports/droute_violations.rpt \
  -output_maze initialization/outputs/gcd_nangate45.output.maze.log \
  -verbose 1

write_def initialization/outputs/gcd_nangate45.def
