set test_case over_v2_bench
set test_dir ../data

read_lef $test_dir/sky130hs.tlef

bench_wires_gen -dbg 2 -over

bench_verilog $test_case.verilog
write_def $test_case.def

