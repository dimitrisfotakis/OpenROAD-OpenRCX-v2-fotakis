set test_case gen_model_3
set d /home/dimitris-ic/z/regression_data/ext/model/data/data.0102.110

set lef $d/patterns.lef
set def $d/patterns.def

set spef1 $d/SPEF/Cmax_125C.spef
set spef2 $d/SPEF/Typ_25C.spef
set spef3 $d/SPEF/Cmin_125C.spef

read_lef $lef
read_def $def 

gen_rcx_model -out_file $test_case.rcx.model -corner_list "MAX TYP MIN"  -spef_file_list "$spef1 $spef2 $spef3"

exit
