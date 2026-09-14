############### using our own tcl scripts while in root directory
# full dumps
~/.local/bin/bazel build //:openroad
setenv DRT_DUMP_GG_DIR "a_star_dumps/nangate45/gcd/ggdump"
setenv DRT_DUMP_EXP_DIR "a_star_dumps/nangate45/gcd/expdump"
setenv DRT_DUMP_COST_DIR "a_star_dumps/nangate45/gcd/costdump"
setenv DRT_DUMP_MIN_ITER 1
setenv DRT_DUMP_ITER 1
./bazel-bin/openroad -no_init -exit initialization/run_gcd.tcl

# single threaded measurements
~/.local/bin/bazel build //:openroad
setenv DRT_DUMP_GG_DIR "a_star_dumps/nangate45/gcd/ggdump"
setenv DRT_DUMP_EXP_DIR ""
setenv DRT_DUMP_COST_DIR ""
setenv DRT_DUMP_MIN_ITER 1
setenv DRT_DUMP_ITER 1
./bazel-bin/openroad -no_init -exit initialization/run_gcd.tcl

# threaded measurements
~/.local/bin/bazel build //:openroad
setenv DRT_DUMP_GG_DIR "top_gcd/ggdump_thread1"
setenv DRT_DUMP_EXP_DIR ""
setenv DRT_DUMP_COST_DIR ""
setenv DRT_DUMP_MIN_ITER 4
setenv DRT_DUMP_ITER 4
./bazel-bin/openroad -no_init -exit initialization/run_aes.tcl


################## using OpenROAD tcl scripts while in test directory
# full dumps
~/.local/bin/bazel build //:openroad
setenv DRT_DUMP_GG_DIR "a_star_dumps/nangate45/gcd/ggdump"
setenv DRT_DUMP_EXP_DIR "a_star_dumps/nangate45/gcd/expdump"
setenv DRT_DUMP_COST_DIR "a_star_dumps/nangate45/gcd/costdump"
setenv DRT_DUMP_MIN_ITER 1
setenv DRT_DUMP_ITER 1
../bazel-bin/openroad -no_init -exit gcd_sky130hs.tcl

# single threaded measurements
~/.local/bin/bazel build //:openroad
setenv DRT_DUMP_GG_DIR "a_star_dumps/nangate45/gcd/ggdump"
setenv DRT_DUMP_EXP_DIR ""
setenv DRT_DUMP_COST_DIR ""
setenv DRT_DUMP_MIN_ITER 1
setenv DRT_DUMP_ITER 1
../bazel-bin/openroad -no_init -exit gcd_sky130hs.tcl

# threaded measurements
~/.local/bin/bazel build //:openroad
setenv DRT_DUMP_GG_DIR "a_star_dumps/nangate45/gcd/ggdump_thread1"
setenv DRT_DUMP_EXP_DIR ""
setenv DRT_DUMP_COST_DIR ""
setenv DRT_DUMP_MIN_ITER 4
setenv DRT_DUMP_ITER 4
../bazel-bin/openroad -no_init -exit gcd_sky130hs.tcl


# asap7 aes aes_asap7.tcl
# asap7 gcd gcd_asap7.tcl
# nangate45 aes aes_nangate45.tcl
# nangate45 gcd gcd_nangate45.tcl
# sky130 aes aes_sky130hd.tcl
# sky130 gcd gcd_sky130hd.tcl
# sky130 ibex ibex_sky130hd.tcl
# sky130 jpeg jpeg_sky130hd.tcl