`timescale 1ns/1ps
module gamecom_sram_model (
    input wire [16:0] a,
    inout wire [15:0] dq,
    input wire oe_n,we_n,ub_n,lb_n
);
    reg [15:0] mem [0:131071];
    real write_start, address_start;
    reg [16:0] write_addr;
    integer writes=0;
    wire [15:0] read_data;
    wire read_enable;
    assign #55 read_data=mem[a];
    assign #20 read_enable=!oe_n && we_n;
    assign dq=read_enable ? read_data : 16'hzzzz;
    always @(a) address_start=$realtime;
    always @(negedge we_n) begin write_start=$realtime; write_addr=a; end
    always @(posedge we_n) if ($realtime>100) begin
        if ($realtime-write_start<45) $fatal(1,"SRAM WE pulse under45ns: start=%0t now=%0t",write_start,$realtime);
        if ($realtime-address_start<50) $fatal(1,"SRAM address setup under50ns");
        if (a!==write_addr) $fatal(1,"SRAM address changed during write");
        if (!lb_n) mem[a][7:0]=dq[7:0];
        if (!ub_n) mem[a][15:8]=dq[15:8];
        writes=writes+1;
    end
endmodule

module gamecom_sram_turnaround_check (
    input wire fpga_drive,oe_n,we_n
);
    wire delayed_oe_n,delayed_fpga_drive,sram_drive;
    assign #(14,0) delayed_oe_n=oe_n;
    assign #(0,14) delayed_fpga_drive=fpga_drive;
    assign #(5,20) sram_drive=!delayed_oe_n && we_n;
    always @(delayed_fpga_drive or sram_drive)
        if ($realtime>100 && delayed_fpga_drive===1'b1 && sram_drive===1'b1)
            $fatal(1,"SRAM bus ownership overlaps at%0t",$realtime);
endmodule

module gamecom_sdram_model #(
    parameter real ACCESS_DELAY_NS=6.0,
    parameter real CLOCK_PERIOD_NS=16.650
) (
    input wire clk,cke,ras_n,cas_n,we_n,
    input wire [12:0] a,
    input wire [1:0] ba,dqm,
    inout wire [15:0] dq
);
    reg [15:0] mem [0:1048575];
    reg [12:0] row [0:3];
    reg [3:0] active=0;
    real activated [0:3];
    real bank_idle [0:3];
    real refreshed=-100000;
    real first_edge=-1;
    reg [19:0] burst_addr;
    reg write_second=0;
    integer read_remaining=0;
    reg [15:0] dq_out;
    reg dq_enable=0;
    integer refreshes=0,reads=0,writes=0,modes=0;
    integer i;
    wire [2:0] command={ras_n,cas_n,we_n};
    assign dq=dq_enable ? dq_out : 16'hzzzz;
    initial for(i=0;i<4;i=i+1) begin activated[i]=-100000; bank_idle[i]=-100000; end
    always @(posedge clk) if (cke) begin
        if (first_edge<0) first_edge=$realtime;
        if (command!=3'b111 && $realtime-first_edge<200000)
            $fatal(1,"SDRAM command before200us initialization");
        if (command!=3'b111 && command!=3'b001 && $realtime-refreshed<80)
            $fatal(1,"SDRAM tRFC under80ns");
        if (modes>=2 && $realtime-refreshed>7812.5)
            $fatal(1,"SDRAM refresh overdue");
        if (write_second) begin
`ifndef VERILATOR
            if (dq===16'hzzzz) $fatal(1,"SDRAM missing second write half");
`endif
            mem[burst_addr+1]=dq;
            writes=writes+1; write_second=0;
        end
        if (read_remaining!=0) begin
            dq_enable<=#(ACCESS_DELAY_NS) 1;
            dq_out<=#(ACCESS_DELAY_NS) mem[burst_addr];
            burst_addr=burst_addr+1;
            read_remaining=read_remaining-1;
        end else dq_enable<=#2.5 0;
        case(command)
            3'b010: begin
                active=0;
                for(i=0;i<4;i=i+1) bank_idle[i]=$realtime+18;
            end
            3'b001: begin
                if ($realtime-refreshed<80) $fatal(1,"Consecutive REF violates80ns");
                for(i=0;i<4;i=i+1) if ($realtime<bank_idle[i])
                    $fatal(1,"Refresh before auto precharge completed");
                if (active!=0) $fatal(1,"Refresh while bank active");
                refreshed=$realtime; refreshes=refreshes+1;
            end
            3'b000: begin
                if(ba==0 && a!==13'h021) $fatal(1,"Unexpected SDRAM mode");
                if(ba==2 && a!==0) $fatal(1,"Unexpected SDRAM extended mode");
                modes=modes+1;
            end
            3'b011: begin
                if ($realtime<bank_idle[ba]) $fatal(1,"ACTIVE before tRP");
                if (active[ba]) $fatal(1,"ACTIVE without precharge");
                row[ba]=a; active[ba]=1; activated[ba]=$realtime;
            end
            3'b100,3'b101: begin
                if (!active[ba]) $fatal(1,"Access without ACTIVE");
                if ($realtime-activated[ba]<18) $fatal(1,"SDRAM tRCD under18ns");
                if (!a[10] || a[0]) $fatal(1,"Expected aligned burst auto-precharge");
                if (row[ba][12:8]!=0) $fatal(1,"Out of2MiB backing range");
                burst_addr={row[ba][7:0],ba,a[9:0]};
                if(command==3'b100) begin
                    if(dqm!==0) $fatal(1,"Unexpected masked SDRAM write");
`ifndef VERILATOR
                    if(dq===16'hzzzz) $fatal(1,"Undriven SDRAM write");
`endif
                    mem[burst_addr]=dq; writes=writes+1; write_second=1;
                    bank_idle[ba]=$realtime+CLOCK_PERIOD_NS+15+18;
                end else begin
                    read_remaining=2; reads=reads+1;
                    bank_idle[ba]=$realtime+2*CLOCK_PERIOD_NS+18;
                end
                if(bank_idle[ba]<activated[ba]+48+18)
                    bank_idle[ba]=activated[ba]+48+18;
                active[ba]=0;
            end
        endcase
    end
endmodule
