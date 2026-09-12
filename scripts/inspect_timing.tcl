package require ::quartus::project
package require ::quartus::sta
project_open -revision ap_core gamecom_pocket
create_timing_netlist
read_sdc
set outdir [file normalize [file join [file dirname [info script]] .. timing-paths]]
file mkdir $outdir
set evidence [open $outdir/cdc-evidence.tsv w]
puts $evidence "VERSION\t3"
puts $evidence "FIT\t[file mtime output_files/ap_core.fit.summary]\t[file size output_files/ap_core.fit.summary]"
flush $evidence

set corners [get_available_operating_conditions]
if {[llength $corners] == 0} { error "No operating corners available for timing analysis" }
set missing_diagnostic_clocks [list]
foreach corner $corners {
    set_operating_conditions $corner
    update_timing_netlist
    report_timing -setup -npaths 20 -nworst 1 -detail full_path -file $outdir/setup-$corner.txt
    report_timing -hold -npaths 20 -nworst 1 -detail full_path -file $outdir/hold-$corner.txt
    report_clock_fmax_summary -file $outdir/fmax-$corner.txt
    foreach {role clock_name} {
        mem {ic|machine_pll|pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
        sys {ic|machine_pll|pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}
        video {ic|machine_pll|pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}
        sdram {sdram_clk}
    } {
        set diagnostic_clocks($role) [get_clocks $clock_name]
        if {[get_collection_size $diagnostic_clocks($role)] == 0} {
            lappend missing_diagnostic_clocks $clock_name
            continue
        }
        foreach analysis {setup hold} {
            report_timing -$analysis -to_clock $diagnostic_clocks($role) -npaths 20 -nworst 1 -detail full_path -file $outdir/$analysis-$role-$corner.txt
            report_timing -$analysis -from_clock $diagnostic_clocks($role) -to_clock $diagnostic_clocks($role) -npaths 20 -nworst 1 -detail full_path -file $outdir/$analysis-$role-same-clock-$corner.txt
        }
    }
    foreach {launch capture} {sdram mem sys mem mem sys} {
        if {[get_collection_size $diagnostic_clocks($launch)] == 0 ||
            [get_collection_size $diagnostic_clocks($capture)] == 0} { continue }
        foreach analysis {setup hold} {
            report_timing -$analysis -from_clock $diagnostic_clocks($launch) -to_clock $diagnostic_clocks($capture) -npaths 64 -nworst 1 -detail full_path -file $outdir/$analysis-$launch-to-$capture-$corner.txt
        }
    }
    foreach analysis {setup hold} {
        report_timing -$analysis -from [get_ports {dram_dq[*]}] -npaths 64 -nworst 1 -detail full_path -file $outdir/$analysis-sdram-read-$corner.txt
        report_timing -$analysis -to [get_ports {dram_dq[*] dram_a[*]}] -npaths 64 -nworst 1 -detail full_path -file $outdir/$analysis-sdram-write-$corner.txt
        report_timing -$analysis -from [get_ports {sram_dq[*]}] -npaths 64 -nworst 1 -detail full_path -file $outdir/$analysis-sram-read-$corner.txt
        report_timing -$analysis -to [get_ports {sram_a[*] sram_dq[*] sram_oe_n sram_we_n sram_ub_n sram_lb_n}] -npaths 64 -nworst 1 -detail full_path -file $outdir/$analysis-sram-write-$corner.txt
    }
    report_path -from [get_ports {sram_dq[*]}] -npaths 16 -file $outdir/sram-input-$corner.txt
}
report_clocks -file $outdir/clocks.txt
report_ucp -file $outdir/unconstrained.txt
if {[llength $missing_diagnostic_clocks] != 0} {
    error "Missing diagnostic clocks: [lsort -unique $missing_diagnostic_clocks]"
}

foreach {bus launch capture minimum} {
    fifo_write {*loader|transfer_fifo|wr_gray[*]} {*loader|transfer_fifo|wr_gray_rd1[*]} 2
    fifo_read {*loader|transfer_fifo|rd_gray[*]} {*loader|transfer_fifo|rd_gray_wr1[*]} 2
    committed {*loader|committed_gray[*]} {*loader|committed_gray_b1[*]} 2
    slot_size {*loader|announced_bytes[*]} {*loader|mem_size[*]} 1
    slot_kind {*loader|active_bios} {*loader|mem_bios} 1
    backing_base {*loader|backing_base[*]} {*loader|mem_base[*]} 1
    expected_crc {*loader|input_crc[*]} {*loader|expected_crc_m1[*]} 1
    result_crc {*loader|mem_result_crc[*]} {*loader|readback_crc32[*]} 1
    runtime_size {*loader|rom_size[*]} {*ic|rom_size_s[*]} 1
    crc_bad_bios {*loader|mem_crc_bad} {*loader|bios_loaded} 1
    crc_bad_cart {*loader|mem_crc_bad} {*loader|cart_loaded} 1
    crc_bad_error {*loader|mem_crc_bad} {*loader|error[*]} 1
    rtc_payload {*rtc_adapter|mailbox_data[*]} {*rtc_adapter|host_time[*]} 1
    audio_payload {*audio_transport|held_sample[*]} {*audio_transport|next_sample[*]} 1
} {
    set sources [get_registers $launch]
    set destinations [get_registers $capture]
    set cdc_sources($bus) $sources
    set cdc_destinations($bus) $destinations
    set source_count [get_collection_size $sources]
    set reachable [dict create]
    foreach_in_collection node [get_fanouts $sources] {
        set node_name [get_node_info -name $node]
        if {![dict exists $reachable $node_name]} {
            dict set reachable $node_name 1
            puts $evidence "FANOUT\t$bus\t$node_name"
        }
    }
    set actual_destinations [list]
    foreach_in_collection node $destinations {
        set node_name [get_object_info -name $node]
        puts $evidence "CANDIDATE\t$bus\t$node_name"
        if {[dict exists $reachable $node_name]} { lappend actual_destinations $node_name }
    }
    set destination_count [llength $actual_destinations]
    if {$source_count < $minimum || $destination_count < $minimum} {
        error "Missing implemented CDC endpoints for $bus: from=$source_count to=$destination_count"
    }
    puts $evidence "ENDPOINT\t$bus\t$source_count\t$destination_count"
    foreach_in_collection node $sources {
        puts $evidence "NODE\t$bus\tfrom\t[get_object_info -name $node]"
    }
    foreach node_name $actual_destinations {
        puts $evidence "NODE\t$bus\tto\t$node_name"
    }
}
set corner_index 0
foreach corner $corners {
    set_operating_conditions $corner
    update_timing_netlist
    puts $evidence "CORNER\t$corner_index\t$corner"
    report_net_delay -file $outdir/net-delay-$corner_index.txt
    foreach bus [array names cdc_sources] {
        set destination_count [get_collection_size $cdc_destinations($bus)]
        report_timing -setup -from $cdc_sources($bus) -to $cdc_destinations($bus) -npaths $destination_count -nworst 1 -detail full_path -file $outdir/cdc-path-$bus-$corner_index.txt
    }
    set skew [report_max_skew -npaths 1 -detail path_only -file $outdir/max-skew-$corner_index.txt]
    if {[llength $skew] != 2 || ![string is integer -strict [lindex $skew 0]] ||
        [lindex $skew 0] < 1 || ![string is double -strict [lindex $skew 1]]} {
        error "Missing or invalid max-skew analysis at $corner: $skew"
    }
    puts $evidence "SKEW\t$corner_index\t[lindex $skew 0]\t[lindex $skew 1]"
    flush $evidence
    incr corner_index
}
if {$corner_index == 0} { error "No operating corners available for CDC timing analysis" }
puts $evidence "COMPLETE\t$corner_index"
close $evidence
project_close
