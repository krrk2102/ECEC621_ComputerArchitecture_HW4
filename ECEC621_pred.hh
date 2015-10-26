/*
 * Copyright (c) 2011 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 *          Timothy M. Jones
 *          
 * Steven Battle
 * sjb328@drexel.edu
 * ECEC-621 gem5 branch predictor project
 * empty class
 *  implement constructor, lookup and update functions
 *  you may add private functions if they will help.
 *  refer to tournament.{cc,hh} and 2bit_local.{cc,hh}
 */

#ifndef __CPU_ECEC621_PRED_HH__
#define __CPU_ECEC621_PRED_HH__

#include <vector>
#include<list>

#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/sat_counter.hh"

/** A struct represents a branch history register, containing tag and recorded history pattern. */
struct HR {
	unsigned int tag;
	unsigned int HistoryPattern;
};

/** A strcut of a single line of set-associative-implemented PHRT. */
struct AHRT {
	/** A set of registers. */
	std::vector<HR> HRs;
	/** A stack to keep track of LRU register, the LRU one is at the back. */
	std::list<unsigned int> LRUstack;
};

/**
 * Implements a predictor for ECEC-621.
 */
class ECEC621BP : public BPredUnit
{
  public:
    /**
     * Default branch predictor constructor.
     */
    ECEC621BP(const Params *params);

    virtual void uncondBranch(void * &bp_history);

    /**
     * Looks up the given address in the branch predictor and returns
     * a true/false value as to whether it is taken.
     * @param branch_addr The address of the branch to look up.
     * @param bp_history Pointer to any bp history state.
     * @return Whether or not the branch is taken.
     *
     * sjb328: implement this
     */
    bool lookup(Addr branch_addr, void * &bp_history);

    /**
     * Updates the branch predictor to Not Taken if a BTB entry is
     * invalid or not found.
     * @param branch_addr The address of the branch to look up.
     * @param bp_history Pointer to any bp history state.
     * @return Whether or not the branch is taken.
     *
     * sjb328: ignore
     */
    void btbUpdate(Addr branch_addr, void * &bp_history);

    /**
     * Updates the branch predictor with the actual result of a branch.
     * @param branch_addr The address of the branch to update.
     * @param taken Whether or not the branch was taken.
     * 
     * sjb328: implement this
     */
    void update(Addr branch_addr, bool taken, void *bp_history, bool squashed);

    // sjb328: ignore
    void squash(void *bp_history)
    { assert(bp_history == NULL); }

    // sjb328: ignore
    void reset();

  private:
    /**
     * Per-address history register table implemented as a set-associative cache.
     */
	
	/** Number of bits that a counter in a PT entry has. */
	unsigned int localCtrBits;
	
	/** Number of entries, or number of counters, a PT has. */
	unsigned int localPredictorSize;
	
	/** Associativity of a set of AHRT. */
	unsigned int set_associativity;
	
	/** Entries of AHRT. */
	unsigned int globalPredictorSize;
	
	/** Input value, product of history pattern bits length and set-associativity. */
	unsigned int globalCtrBits;
	
	/** Mask for AHRT look up using an address. */
	unsigned int AHRTlookupMask;
	
	/** Number of bits for looking up AHRT using an address. */
	unsigned int AHRTindexLength;
	
	/** History pattern register bits length. */
	unsigned int hp_length;
	
	/** Mask for history pattern. */
	unsigned int hp_mask;
	
	/** Set-associative history register table, or per-address history table. */
	std::vector<AHRT> AHRTs;
	
	/** History pattern table. */
	std::vector<SatCounter> PT;
	
	/** Number of bits to shift the instruction over to get rid of the word offset. */
	unsigned instShiftAmt;
	
	/** Using index and tag to access AHRT. */
	unsigned int AHRTread(unsigned int _index, unsigned int _tag);
	
	/** Update AHRT content. */
	unsigned int AHRTupdate(unsigned int _index, unsigned int _tag, bool taken);
	
	/** Using pattern from AHRT to look up predictor value. */
	bool PTread(unsigned int _pattern);
};

#endif // __CPU_ECEC621_PRED_HH__
