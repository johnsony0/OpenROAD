read_lef "test/Nangate45/Nangate45_tech.lef"
read_lef "test/Nangate45/Nangate45_stdcell.lef"
read_def "src/drt/test/gcd_nangate45_preroute.def"
read_guides "src/drt/test/gcd_nangate45.route_guide"

detailed_route_debug \
  -dr \
  -maze \
  -pin {_573_/ZN} \
  -box {50400 50400 79800 79800} \
  -iter 1 \
  -dump_dr \
  -dump_dir initialization/dr_debug

detailed_route -output_drc initialization/reports/droute_violations.rpt \
  -output_maze initialization/outputs/gcd_nangate45.output.maze.log \
  -verbose 1