# Starting the Container
# docker start openroad_sandbox
# docker exec -it openroad_sandbox bash

# openroad -no_init -exit initialization/run_drt.tcl

# Read LEF files 
read_lef test/Nangate45/Nangate45_tech.lef
read_lef test/Nangate45/Nangate45_stdcell.lef

# Read globally-routed design with guides
# read_def src/drt/test/aes_nangate45_preroute.def
read_def src/drt/test/gcd_nangate45_preroute.def

read_guides src/drt/test/gcd_nangate45.route_guide

detailed_route_debug -dump_dr -dump_dir initialization/debugs -iter 2

# Run detailed routing (TritonRoute)
detailed_route -output_guide initialization/outputs/gcd_nangate45.output.guide.mod \
               -output_drc initialization/reports/droute_violations.rpt \
               -output_maze initialization/outputs/gcd_nangate45.output.maze.log \
               -drc_report_iter_step 1 \
               -verbose 1

# Save the routing-completed DEF
write_def initialization/outputs/output_detailed_routed.def

exit