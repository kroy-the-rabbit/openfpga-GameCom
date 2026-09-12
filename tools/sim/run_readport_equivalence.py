#!/usr/bin/env python3
"""Exhaustively compare the CPU's shared read routing with its original ports."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CPU = Path(os.environ.get("READPORT_CPU_SOURCE", ROOT / "src/fpga/gamecom/rtl/sm8521.v"))
source = CPU.read_text()
classes = dict(re.findall(r"localparam\s+(CLASS_\w+)\s*=\s*6'd(\d+);", source))
assert len(set(classes.values())) == len(classes), "Opcode classes are not mutually exclusive"

consumer = None
operand_classes, target_classes = set(), set()
execute = source[source.index("\n" + "\t" * 6 + "case (op_class_q)"):]
for line in execute.splitlines():
    match = re.match(r"^\t{7}(CLASS_\w+)\s*:", line)
    if match:
        consumer = match[1]
    if re.search(r"\brd_op1(?:_hi)?_v\b", line):
        assert consumer is not None
        operand_classes.add(consumer)
    if re.search(r"\brd_tgt(?:_hi)?_v\b", line):
        assert consumer is not None
        target_classes.add(consumer)
assert target_classes == {"CLASS_MOVM_MASK", "CLASS_WORD_IMM_OP"}
assert operand_classes == {"CLASS_DIRECT_IMM_OP", "CLASS_MUL_RR", "CLASS_MUL_IMM",
                           "CLASS_DIV_RR", "CLASS_DIV_IMM", "CLASS_BMOV_BF",
                           "CLASS_BF_LOGIC", "CLASS_WORD_RR_OP", "CLASS_FIXED_BYTE_OP"}
assert not target_classes & operand_classes

start = source.rindex("\n\t\tcase (op_class_q)", 0, source.index("\n\t\trd_op0_v    ="))
end = source.index("\n", source.index("\t\trd_ind_v    =", start))
routing = source[start:end]
assert len(re.findall(r"\bdirect_read\(", routing)) == 5, "The two read decoders were not shared"
parameters = "\n".join(f"localparam {name}=6'd{value};" for name, value in classes.items())
module = """
module shared_reads(
    input [5:0] op_class_q,
    input [7:0] operand0_q, operand1_q, target_addr_q, indirect_addr_v,
    output reg [7:0] rd_op0_v,rd_op0_hi_v,rd_op1_v,rd_op1_hi_v,
                     rd_tgt_v,rd_tgt_hi_v,rd_ind_v);
    reg [7:0] rd_op1_addr_v;
    function [7:0] direct_read;
        input [7:0] address;
        begin direct_read=address; end
    endfunction
PARAMETERS
    always @* begin
ROUTING
    end
endmodule
""".replace("PARAMETERS", parameters).replace("ROUTING", routing)
bench = """
module tb_readports;
    reg [5:0] cls;
    reg [7:0] op0,op1,target,indirect;
    wire [7:0] op0_value,op0_next,op1_value,op1_next,target_value,target_next,indirect_value;
    shared_reads dut(cls,op0,op1,target,indirect,op0_value,op0_next,
                     op1_value,op1_next,target_value,target_next,indirect_value);
    // Independent reference: these are the original seven address expressions.
    wire [7:0] ref_op0=op0,ref_op0_next=op0+8'h01;
    wire [7:0] ref_op1=op1,ref_op1_next=op1+8'h01;
    wire [7:0] ref_target=target,ref_target_next=target+8'h01;
    wire [7:0] ref_indirect=indirect;
    integer c,a,b,checks=0;
    initial begin
        for(c=0;c<64;c=c+1) begin
            for(a=0;a<256;a=a+1) begin
                for(b=0;b<256;b=b+1) begin
                    cls=c;op1=a;target=b;op0=a^b^8'h5a;indirect=b+c;
                    #1;
                    if({op0_value,op0_next,indirect_value} !== {ref_op0,ref_op0_next,ref_indirect})
                        $fatal(1,"Unshared read port changed class=%0d op1=%h target=%h",c,op1,target);
                    if(c==MOVM || c==WORD_IMM) begin
                        if(target_value!==ref_target || (c==WORD_IMM && target_next!==ref_target_next))
                            $fatal(1,"Target read mismatch class=%0d op1=%h target=%h",c,op1,target);
                    end else if({op1_value,op1_next} !== {ref_op1,ref_op1_next})
                        $fatal(1,"Operand1 read mismatch class=%0d op1=%h target=%h",c,op1,target);
                    checks=checks+1;
                end
            end
        end
        if(checks!=4194304) $fatal(1,"Incomplete address/class sweep");
        $display("PASS read-port reference equivalence: %0d exhaustive class/address cases",checks);
        $finish;
    end
endmodule
""".replace("MOVM", classes["CLASS_MOVM_MASK"]).replace("WORD_IMM", classes["CLASS_WORD_IMM_OP"])

with tempfile.TemporaryDirectory(prefix="gamecom-readports-") as temporary:
    temporary = Path(temporary)
    def run(rtl, label):
        design = temporary / f"{label}.sv"
        executable = temporary / f"{label}.vvp"
        design.write_text(rtl + bench)
        subprocess.run(["iverilog", "-g2012", "-s", "tb_readports", "-o", str(executable), str(design)], check=True)
        return subprocess.run(["vvp", str(executable)], capture_output=True, text=True, timeout=120)
    result = run(module, "actual")
    if result.returncode:
        raise SystemExit(result.stdout + result.stderr)
    print(result.stdout.strip())
    broken = module.replace("CLASS_MOVM_MASK, CLASS_WORD_IMM_OP:", "CLASS_MOVM_MASK, CLASS_WORD_RR_OP:")
    assert broken != module
    result = run(broken, "wrong-selector")
    assert result.returncode != 0 and "read mismatch" in result.stdout, result.stdout + result.stderr
print("PASS consumer exclusivity: nine operand1 classes, two target classes, wrong-class mutation rejected")
