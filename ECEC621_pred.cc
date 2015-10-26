/*
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
 *
 * Steven Battle
 * sjb328@drexel.edu
 * ECECC-621 gem5 branch predictor project
 * empty class
 *  implement constructor, lookup and update functions
 *  you may add private functions if they will help.
 *  refer to tournament.{cc,hh} and 2bit_local.{cc,hh}
 */

#include <cmath>
#include <limits>

#include "base/intmath.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "cpu/pred/ECEC621_pred.hh"
#include "debug/Fetch.hh"

ECEC621BP::ECEC621BP(const Params *params)
    : BPredUnit(params),
	  localCtrBits(params->localCtrBits),
	  localPredictorSize(params->localPredictorSize),
	  globalPredictorSize(params->globalPredictorSize),
	  globalCtrBits(params->globalCtrBits),
	  instShiftAmt(params->instShiftAmt)
      // you may want to add some configuration parameters.
      // refer to tournament.cc and 2bit_local.cc
{
    DPRINTF(Fetch, "ECECC-621 branch predictor");
	// Add your objects and store configuration parameters
	if (!isPowerOf2(localPredictorSize)) {
		fatal("Invalid pattern table size!\n");
	}
	if (!isPowerOf2(globalPredictorSize)) {
		fatal("Invalid number of history register table entries!\n");
	}
	// Setting up numbers useful for bits operations.
	hp_length = (unsigned int)log2(localPredictorSize);
	hp_mask = localPredictorSize - 1;
	set_associativity = globalCtrBits / hp_length;
	AHRTlookupMask = globalPredictorSize - 1;
	AHRTindexLength = log2(globalPredictorSize);
	
	// Initializing AHRT, set up history registers and its tag, also LRU tracker.
	AHRTs.resize(globalPredictorSize);
	for (unsigned int i = 0; i < globalPredictorSize; i++) {
		AHRTs[i].HRs.resize(set_associativity);
		for (unsigned int j = 0; j < set_associativity; j++) {
			AHRTs[i].HRs[j].tag = std::numeric_limits<unsigned int>::max();
			AHRTs[i].HRs[j].HistoryPattern = 0;
			AHRTs[i].LRUstack.push_front(j);
		}
	}
	
	// Initializing pattern table.
	PT.resize(localPredictorSize);
	for (unsigned int i = 0; i < localPredictorSize; i++) {
		PT[i].setBits(localCtrBits);
	}
}

void
ECEC621BP::reset()
{
// ignore
}

void
ECEC621BP::btbUpdate(Addr branch_addr, void * &bp_history)
{
// Place holder for a function that is called to update predictor history when
// a BTB entry is invalid or not found.
//
// ignore
}

unsigned int
ECEC621BP::AHRTread(unsigned int _index, unsigned int _tag)
{
	if (_index >= AHRTs.size()) {
		fatal("Out of AHRT bound!\n");
	}
	for (unsigned int i = 0; i < set_associativity; i++) {
		if (AHRTs[_index].HRs[i].tag == _tag) {
			return AHRTs[_index].HRs[i].HistoryPattern;
		}
	}
	return std::numeric_limits<unsigned int>::max();
}

bool
ECEC621BP::PTread(unsigned int _pattern)
{
	if (_pattern == std::numeric_limits<unsigned int>::max()) {
		return true;
	}
	if (_pattern >= PT.size()) {
		fatal("Out of PT bound!\n");
	}
	return (PT[_pattern].read()>>(localCtrBits-1));
}

bool
ECEC621BP::lookup(Addr branch_addr, void * &bp_history)
{
    // implement the branch prediction lookup
	unsigned int AHRTIndex;
	unsigned int AHRTtag;
	unsigned int pattern;
	bool prediction;
	
	branch_addr = branch_addr >> instShiftAmt;
	// Last few bits is used as table index.
	AHRTIndex = branch_addr & AHRTlookupMask;
	// Creating tag.
	AHRTtag = branch_addr >> AHRTindexLength;
	// Read history pattern from AHRT.
	pattern = AHRTread(AHRTIndex, AHRTtag);
	// Read predictor value from PT.
	prediction = PTread(pattern);
	
	return prediction;
}

unsigned int
ECEC621BP::AHRTupdate(unsigned int _index, unsigned int _tag, bool taken)
{
	if (_index >= AHRTs.size()) {
		fatal("Out of AHRT bound while updating!\n");
	}
	unsigned int lastpattern;
	for (std::list<unsigned int>::iterator it = AHRTs[_index].LRUstack.begin(); it != AHRTs[_index].LRUstack.end(); ++it) {
		if (AHRTs[_index].HRs[*it].tag == _tag) {
			lastpattern = AHRTs[_index].HRs[*it].HistoryPattern;
			if (taken) {
				AHRTs[_index].HRs[*it].HistoryPattern = ((AHRTs[_index].HRs[*it].HistoryPattern<<1)|1) & hp_mask;
			} else {
				AHRTs[_index].HRs[*it].HistoryPattern = (AHRTs[_index].HRs[*it].HistoryPattern<<1) & hp_mask;
			}
			AHRTs[_index].LRUstack.push_front(*it);
			it = AHRTs[_index].LRUstack.erase(it);
			return lastpattern;
		}
	}
	AHRTs[_index].HRs[AHRTs[_index].LRUstack.back()].tag = _tag;
	lastpattern = std::numeric_limits<unsigned int>::max();
	if (taken) {
		AHRTs[_index].HRs[AHRTs[_index].LRUstack.back()].HistoryPattern = 1;
	} else {
		AHRTs[_index].HRs[AHRTs[_index].LRUstack.back()].HistoryPattern = 0;
	}
	AHRTs[_index].LRUstack.push_front(AHRTs[_index].LRUstack.back());
	AHRTs[_index].LRUstack.pop_back();
	return lastpattern;
}

void
ECEC621BP::update(Addr branch_addr, bool taken, void *bp_history, bool squashed)
{
    // implement the branch prediction update
	unsigned int AHRTIndex;
	unsigned int AHRTtag;
	unsigned int pattern;

	branch_addr = branch_addr >> instShiftAmt;
	// Last few bits is used as table index.
	AHRTIndex = branch_addr & AHRTlookupMask;
	// Creating tag.
	AHRTtag = branch_addr >> AHRTindexLength;
	// Read history pattern from AHRT, as table entry for PT, and update AHRT.
	pattern = AHRTupdate(AHRTIndex, AHRTtag, taken);
	if (pattern != std::numeric_limits<unsigned int>::max()) {
		if (taken) {
			DPRINTF(Fetch, "Branch updated as taken.\n");
			if (pattern >= PT.size()) {
				fatal("Out of PT bound, updating as taken!\n");
			}
			PT[pattern].increment();
		} else {
			DPRINTF(Fetch, "Branch updated as not taken.\n");
			if (pattern >= PT.size()) {
				fatal("Out of PT bound, updating as not taken!\n");
			}
			PT[pattern].decrement();
		}
	}
}


void
ECEC621BP::uncondBranch(void *&bp_history)
{
// ignore
}
