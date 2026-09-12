#include "Vgamecom_boot_sim.h"
#include "verilated.h"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <set>
#include <sstream>
#include <string>
#include <vector>

static std::vector<uint8_t> read_file(const char *path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error(std::string("Cannot read ") + path);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
static void write_ppm(const std::string &path,const std::vector<uint32_t> &pixels) {
    std::ofstream ppm(path,std::ios::binary);
    ppm << "P6\n200 160\n255\n";
    for(uint32_t rgb:pixels) for(int shift:{16,8,0}) ppm.put(static_cast<char>((rgb>>shift)&255));
}

int main(int argc, char **argv) {
    if (argc != 8) { std::cerr << "bios.bin game.tgc milliseconds frame.ppm keys.csv rom_wait_cycles live_keys\n"; return 2; }
    auto bios=read_file(argv[1]), rom=read_file(argv[2]);
    if (bios.size()!=262144 || rom.empty()) return 2;
    const uint64_t ticks=std::stoull(argv[3])*120000;
    const unsigned rom_wait_cycles=std::stoul(argv[6]);
    const std::string live_keys=argv[7];
    VerilatedContext context;
    context.commandArgs(argc,argv);
    Vgamecom_boot_sim m(&context);
    uint64_t bios_reads=0, cart_reads=0, vram_writes=0, frames=0;
    uint64_t rom_wait_clocks=0, rom_cache_misses=0;
    uint64_t address_changes_pending=0;
    uint64_t touch_scan_clocks=0, simulated_ticks=0;
    std::vector<uint32_t> frame, last_frame;
    frame.reserve(32000);
    int amin=32767,amax=-32768;
    unsigned previous_sys=0, previous_vid=0;
    std::ifstream events_file(argv[5]);
    std::vector<std::pair<uint64_t,uint32_t>> events;
    uint64_t event_ms;uint32_t event_keys;
    while(events_file >> event_ms >> std::hex >> event_keys >> std::dec)
        events.emplace_back(event_ms*120000,event_keys);
    size_t event_index=0;
    uint32_t keys=0;
    std::vector<std::pair<uint64_t,uint32_t>> applied_keys;
    unsigned wait_remaining=0;
    uint32_t cache_tag=0,pending_tag=0;
    bool cache_valid=false,pending=false;
    for (uint64_t t=0;t<ticks;++t) {
        // No APF commands are sent in this harness. Its otherwise idle bridge
        // clock can follow clk_sys; the RTC mailbox has an asynchronous unit
        // test. Skip slots containing no system/video edge to reduce runtime.
        if(t && t%2 && t%3) continue;
        m.reset_cold=(t<120);
        m.reset_run=m.reset_cold || !m.ram_initialized;
        while(event_index<events.size() && t>=events[event_index].first)
            keys=events[event_index++].second;
        // Optional interactive diagnostic control. "auto" keeps the scripted
        // inputs, a hexadecimal mask overrides them, and "stop" ends cleanly.
        // Poll at 10 simulated ms and record the actual applied event times so
        // every interactive session can be replayed as an ordinary key script.
        if(!live_keys.empty() && t%1200000==0) {
            std::ifstream live(live_keys);std::string token;live>>token;
            if(token=="stop") break;
            if(!token.empty() && token!="auto") keys=std::stoul(token,nullptr,16);
        }
        if((applied_keys.empty() && keys) || (!applied_keys.empty() && keys!=applied_keys.back().second))
            applied_keys.emplace_back(t/120000,keys);
        m.keys=keys;
        m.clk_sys=(t/3)&1;
        m.clk_vid=(t/2)&1;
        m.clk_bridge=m.clk_sys;
        const unsigned a=m.cart_addr;
        const unsigned backing=(rom.size()==0x1c0000) ? a : (a&(rom.size()-1));
        m.cart_data=255;
        m.rom_ready=1;
        if(a<0x40000) m.cart_data=bios[a];
        else if(m.slot1) {
            if(rom.size()==0x1c0000) m.cart_data=rom[a-0x40000];
            else m.cart_data=rom[a&(rom.size()-1)];
        }
        if(m.reset_run) {cache_valid=false;pending=false;}
        else if(rom_wait_cycles) {
            // One cached 32-bit word and a held request, as in the Pocket
            // bridge. Only cartridge READY waits; BIOS and all clocks continue.
            if(m.clk_sys && !previous_sys) {
                if(pending) {
                    if((backing>>2)!=pending_tag) {
                        if(address_changes_pending<4) std::cerr << "Pending ROM response at " << t/120000.0
                            << " ms: tag=" << std::hex << pending_tag << " live_address=" << a
                            << " pc=" << unsigned(m.debug_pc) << " state=" << std::dec << unsigned(m.debug_state)
                            << " rd=" << unsigned(m.cart_rd) << " slot=" << unsigned(m.slot1) << "\n";
                        ++address_changes_pending;
                    }
                    if(--wait_remaining==0) {cache_tag=pending_tag;cache_valid=true;pending=false;}
                } else if(m.cart_rd && a>=0x40000 && m.slot1 && (!cache_valid || cache_tag!=(backing>>2))) {
                    pending_tag=backing>>2;pending=true;wait_remaining=rom_wait_cycles;
                    ++rom_cache_misses;
                }
            }
            if(m.cart_rd && a>=0x40000 && m.slot1)
                m.rom_ready=cache_valid && cache_tag==(backing>>2);
            if(!m.rom_ready) m.cart_data=255;
        }
        m.eval();
        if(m.clk_sys && !previous_sys && !m.reset_run) {
            if(m.cart_rd) { if(m.cart_addr<0x40000) ++bios_reads; else ++cart_reads; }
            if(!m.rom_ready) ++rom_wait_clocks;
            if(m.debug_vram_write) ++vram_writes;
            if(m.debug_touch_active && m.debug_touch_scan==(0x3fff & ~(1u<<m.debug_touch_x))) ++touch_scan_clocks;
            int sample=static_cast<int16_t>(m.audio_sample);
            amin=std::min(amin,sample);amax=std::max(amax,sample);
        }
        if(m.clk_vid && !previous_vid && !m.video_skip && !m.reset_run) {
            if(m.video_vs) {
                if(frame.size()==32000) {
                    last_frame=frame;++frames;
                    if(frames%60==0) {
                        std::string stem=argv[4];stem.resize(stem.size()-4);
                        write_ppm(stem+"-"+std::to_string(t/120000)+".ppm",last_frame);
                        std::ofstream progress(stem+"-progress.json");
                        progress << "{\"milliseconds\":" << t/120000 << ",\"touch_x\":" << unsigned(m.debug_touch_x)
                            << ",\"touch_y\":" << unsigned(m.debug_touch_y) << ",\"touch_active\":" << unsigned(m.debug_touch_active)
                            << ",\"touch_scan_clocks\":" << touch_scan_clocks << ",\"pc\":" << unsigned(m.debug_pc)
                            << ",\"state\":" << unsigned(m.debug_state) << "}\n";
                    }
                }
                frame.clear();
            }
            if(m.video_de) frame.push_back(m.video_rgb);
        }
        previous_sys=m.clk_sys;previous_vid=m.clk_vid;
        simulated_ticks=t+1;
        context.timeInc(1);
    }
    m.final();
    uint32_t fnv=2166136261u;
    std::set<uint32_t> colors(last_frame.begin(),last_frame.end());
    if(!last_frame.empty()) {
        std::ofstream ppm(argv[4],std::ios::binary);
        ppm << "P6\n200 160\n255\n";
        for(uint32_t rgb:last_frame) for(int shift:{16,8,0}) {
            const uint8_t b=(rgb>>shift)&255;
            ppm.put(static_cast<char>(b));fnv=(fnv^b)*16777619u;
        }
    }
    std::cout << "{\"milliseconds\":" << (simulated_ticks+119999)/120000 << ",\"rom_wait_cycles\":" << rom_wait_cycles << ",\"bios_read_clocks\":" << bios_reads
        << ",\"rom_wait_clocks\":" << rom_wait_clocks << ",\"rom_cache_misses\":" << rom_cache_misses
        << ",\"address_changes_pending\":" << address_changes_pending
        << ",\"cart_read_clocks\":" << cart_reads << ",\"vram_write_clocks\":" << vram_writes
        << ",\"frames\":" << frames << ",\"colors\":" << colors.size()
        << ",\"frame_fnv1a\":\"" << std::hex << std::setw(8) << std::setfill('0') << fnv
        << "\",\"pc\":\"" << std::setw(4) << unsigned(m.debug_pc)
        << "\",\"opcode\":\"" << std::setw(2) << unsigned(m.debug_opcode)
        << "\",\"state\":" << std::dec << unsigned(m.debug_state)
        << ",\"audio_min\":" << amin << ",\"audio_max\":" << amax
        << ",\"touch_scan_clocks\":" << touch_scan_clocks << ",\"applied_keys\":[";
    for(size_t i=0;i<applied_keys.size();++i) {
        if(i) std::cout << ',';
        std::cout << '[' << applied_keys[i].first << ',' << applied_keys[i].second << ']';
    }
    std::cout << "]}\n";
    return (bios_reads && vram_writes && frames>1 && colors.size()>1 && address_changes_pending==0) ? 0 : 1;
}
