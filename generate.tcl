package require ::quartus::project
package require ::quartus::flow
set base_dir [pwd]
project_open -revision ap_core src/fpga/build/gamecom_pocket.qpf
set_global_assignment -name NUM_PARALLEL_PROCESSORS 4
execute_flow -compile
project_close
cd $base_dir
