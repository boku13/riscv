//-----------------------------------------------------------------
//                     RISC-V ISA Simulator 
//                            V1.0
//                     Ultra-Embedded.com
//                     Copyright 2014-2017
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>

#include "riscv.h"
#include "elf_load.h"
#include "cosim_api.h"

#include "riscv_main.h"

//-----------------------------------------------------------------
// mem_create: Create memory region
//-----------------------------------------------------------------
static int mem_create(void *arg, uint32_t base, uint32_t size)
{
    return cosim::instance()->create_memory(base, size);
}
//-----------------------------------------------------------------
// mem_load: Load byte into memory
//-----------------------------------------------------------------
static int mem_load(void *arg, uint32_t addr, uint8_t data)
{
    cosim::instance()->write(addr, data);
    return cosim::instance()->valid_addr(addr);
}
//-----------------------------------------------------------------
// riscv_main
//-----------------------------------------------------------------
int riscv_main(cosim_cpu_api *sim, int argc, char *argv[])
{
    unsigned _cycles = 0;
    int max_cycles = -1;
    char *filename = NULL;
    int help = 0;
    int trace = 0;
    uint32_t trace_mask = 1;
    uint32_t stop_pc = 0xFFFFFFFF;
    uint32_t trace_pc = 0xFFFFFFFF;
    uint32_t mem_base = 0x00000000;
    uint32_t mem_size = (32 * 1024 * 1024);
    bool explicit_mem = false;
    char *   dump_file      = NULL;
    char *   dump_sym_start = NULL;
    char *   dump_sym_end   = NULL;
    int      branch_predictor_mode = 1;  // Default: static prediction
    uint32_t opt_bht_size = 256;     // default
    uint32_t opt_btb_size = 64;      // default 
    uint8_t  opt_ghr_bits = 8;       // default 
    uint32_t opt_ras_size = 16;      // default
    uint32_t opt_load_latency = 0;   // default
    uint32_t opt_icache_miss_penalty = 0; // default
    int c; 

    while ((c = getopt (argc, argv, "t:v:f:c:r:d:b:s:e:p:j:k:B:H:T:G:R:L:M:")) != -1)
    {
        switch(c)
        {
            case 't':
                trace = strtoul(optarg, NULL, 0);
                break;
            case 'v':
                trace_mask = strtoul(optarg, NULL, 0);
                break;
            case 'r':
                stop_pc = strtoul(optarg, NULL, 0);
                break;
            case 'f':
                filename = optarg;
                break;
            case 'c':
                max_cycles = (int)strtoul(optarg, NULL, 0);
                break;
            case 'b':
                mem_base = strtoul(optarg, NULL, 0);
                explicit_mem = true;
                break;
            case 's':
                mem_size = strtoul(optarg, NULL, 0);
                explicit_mem = true;
                break;
            case 'e':
                trace_pc = strtoul(optarg, NULL, 0);
                break;
            case 'p':
                dump_file = optarg;
                break;
            case 'j':
                dump_sym_start = optarg;
                break;
            case 'k':
                dump_sym_end = optarg;
                break;
            case 'B':
                branch_predictor_mode = strtoul(optarg, NULL, 0);
                break;
            case 'H': opt_bht_size = strtoul(optarg, NULL, 0); break;
            case 'G': opt_ghr_bits = strtoul(optarg, NULL, 0); break;
            case 'T': opt_btb_size = strtoul(optarg, NULL, 0); break;
            case 'R': opt_ras_size = strtoul(optarg, NULL, 0); break;
            case 'L': opt_load_latency = strtoul(optarg, NULL, 0); break;
            // TODO: Add flag for cache miss penalty if needed, or reuse/add another flag
            // For now, let's assume we might want to add it or just stick to what was requested.
            // The user request mentioned "Cache miss penalties", but didn't specify a flag letter.
            // I'll add 'M' for miss penalty.
            case 'M': opt_icache_miss_penalty = strtoul(optarg, NULL, 0); break;
            case '?':
            default:
                help = 1;   
                break;
        }
    }

    if (help || filename == NULL)
    {
        fprintf (stderr,"Usage:\n");
        fprintf (stderr,"-f filename.elf = Executable to load (ELF)\n");
        fprintf (stderr,"-t [0/1]        = Enable program trace\n");
        fprintf (stderr,"-v 0xX          = Trace Mask\n");
        fprintf (stderr,"-c nnnn         = Max instructions to execute\n");
        fprintf (stderr,"-r 0xnnnn       = Stop at PC address\n");
        fprintf (stderr,"-e 0xnnnn       = Trace from PC address\n");
        fprintf (stderr,"-b 0xnnnn       = Memory base address (for binary loads)\n");
        fprintf (stderr,"-s nnnn         = Memory size (for binary loads)\n");
        fprintf (stderr,"-B [0-5]        = Branch predictor mode (0=none, 1=static, 2=1-bit, 3=2-bit, 4=GShare, 5=BTB+GShare)\n");
        fprintf (stderr,"-H nnnn         = BHT size (entries, power of two)\n");
        fprintf (stderr,"-T nnnn         = BTB size (entries, power of two)\n");
        fprintf (stderr,"-G n            = GHR bits (1-16)\n");
        fprintf (stderr,"-R nnnn         = RAS size (entries)\n");
        fprintf (stderr,"-L n            = Load-use latency (cycles)\n");
        fprintf (stderr,"-M n            = I-Cache miss penalty (cycles)\n");
        fprintf (stderr,"-p dumpfile.bin = Post simulation memory dump file\n");
        fprintf (stderr,"-j sym_name     = Symbol for memory dump start\n");
        fprintf (stderr,"-k sym_name     = Symbol for memory dump end\n");
        exit(-1);
    }

    if (explicit_mem)
    {
        printf("MEM: Create memory 0x%08x-%08x\n", mem_base, mem_base + mem_size-1);
        mem_create(NULL, mem_base, mem_size);
    }

    uint32_t start_addr = 0;

    // Load ELF file
    if (elf_load(filename, mem_create, mem_load, sim, &start_addr))
    {
        printf("Starting from 0x%08x\n", start_addr);

        // Register dump handler
        if (dump_file)
        {
            cosim::instance()->dump_on_exit(dump_file, 
                         (uint32_t)elf_get_symbol(filename, dump_sym_start),
                         (uint32_t)elf_get_symbol(filename, dump_sym_end));
        }

        // Reset CPU to given start PC
        sim->reset(start_addr);

        // Set branch predictor mode (cast to Riscv* to access extended API)
        ((Riscv*)sim)->set_branch_predictor(branch_predictor_mode);
        // Apply microarchitectural configuration
        ((Riscv*)sim)->set_bht_size(opt_bht_size);
        ((Riscv*)sim)->set_btb_size(opt_btb_size);
        ((Riscv*)sim)->set_ghr_bits(opt_ghr_bits);
        ((Riscv*)sim)->set_ras_size(opt_ras_size);
        ((Riscv*)sim)->set_load_latency(opt_load_latency);
        ((Riscv*)sim)->set_icache_params(opt_icache_miss_penalty > 0, opt_icache_miss_penalty);

        // Enable trace?
        if (trace)
            sim->enable_trace(trace_mask);

        _cycles = 0;

        // Start wall-clock timer
        struct timespec start_time, end_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        uint32_t current_pc = 0;
        while (!sim->get_fault() && !sim->get_stopped() &&  current_pc != stop_pc)
        {
            current_pc = sim->get_pc();
            sim->step();
            _cycles++;

            if (max_cycles != -1 && max_cycles == _cycles)
                break;

            // Turn trace on
            if (trace_pc == current_pc)
                sim->enable_trace(trace_mask);
        }   

        // End wall-clock timer
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        
        // Calculate elapsed time in seconds
        double elapsed = (end_time.tv_sec - start_time.tv_sec) + 
                        (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
        
        // Dump stats
        ((Riscv*)sim)->stats_dump();

        printf("\n=== Simulation Performance ===\n");
        printf("Wall-clock time: %.3f seconds\n", elapsed);
        printf("Simulation speed: %.2f MIPS (Million Instructions Per Second)\n", 
               _cycles / elapsed / 1e6);

        cosim::instance()->at_exit(sim->get_fault());
    }
    else
        fprintf (stderr,"Error: Could not open %s\n", filename);

    // Fault occurred?
    if (sim->get_fault())
        return 1;
    else
        return 0;
}
