top_dir1=$(shell pwd)
top_dir=$(top_dir1)
#top_dir=/home/dimitris-ic/z/72424/RegressionTest/
data_dir=$(top_dir)/data

TOP_DIR=/home/dimitris-ic/z/v2_rcx/OpenROAD-OpenRCX-v2-fotakis/
test_dir=$(TOP_DIR)/src/rcx/test/
# or_exec=$(TOP_DIR)/build/src/openroad
or_exec=/home/dimitris-ic/z/openRCX_v2/OpenROAD-OpenRCX-v2-fotakis/build/src/openroad
run_rcx=./scripts/run_rcx.bash
gen_model_rcx=./scripts/run_rcx_gen_model.bash
gold_dir=/home/dimitris-ic/z/regression_data/ext/model/GOLD

.PHONY : v1_patterns v12_patterns over_v2_bench over_under_v2_bench under_v2_bench gen_model_3

.SILENT:

patterns: v1_patterns v12_patterns gen_model_3

bench: over_v2_bench over_under_v2_bench under_v2_bench 

v1_patterns: 
		$(run_rcx) $(or_exec) ./ $@ $(gold_dir)

v12_patterns: 
		$(run_rcx) $(or_exec) ./ $@ $(gold_dir)

gen_model_3:
		$(gen_model_rcx) $(or_exec) ./ $@ $(gold_dir)/$@

over_v2_bench: 
		$(run_rcx) $(or_exec) ./ $@ $(gold_dir)

over_under_v2_bench: 
		$(run_rcx) $(or_exec) ./ $@ $(gold_dir)

under_v2_bench: 
		$(run_rcx) $(or_exec) ./ $@ $(gold_dir)

