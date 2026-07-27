# bazelisk build //:openroad
# OR="$(bazelisk info -c opt bazel-bin)/openroad"
# "$OR" -version
# export DRT_DUMP_GG_DIR="work/ggdump"
# export DRT_DUMP_EXP_DIR="work/expdump"
# "$OR" -no_init -exit initialization/run_top_level_term.tcl

read_lef "test/sky130hs/sky130hs.tlef"
read_lef "test/sky130hs/sky130_fd_sc_hs_merged.lef"
read_def "src/drt/test/top_level_term.def"

read_guides "src/drt/test/top_level_term.guide"

set_routing_layers -signal met1-met3
detailed_route -verbose 0

write_def initialization/outputs/top_level_term.def