# bazelisk build //:openroad
# OR="$(bazelisk info -c opt bazel-bin)/openroad"
# "$OR" -version
# export DRT_DUMP_GG_DIR="top_gcd/ggdump"
# export DRT_DUMP_EXP_DIR="top_gcd/expdump"
# export DRT_DUMP_COST_DIR="top_gcd/costdump"
# "$OR" -no_init -exit initialization/run_gcd.tcl

read_lef "test/Nangate45/Nangate45_tech.lef"
read_lef "test/Nangate45/Nangate45_stdcell.lef"
read_def "src/drt/test/gcd_nangate45_preroute.def"
read_guides "src/drt/test/gcd_nangate45.route_guide"

detailed_route -output_drc initialization/reports/droute_violations.rpt \
  -output_maze initialization/outputs/gcd_nangate45.output.maze.log \
  -verbose 1

write_def initialization/outputs/gcd_nangate45.def