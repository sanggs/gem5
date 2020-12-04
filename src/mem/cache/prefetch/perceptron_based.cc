/**
 * Copyright (c) 2018 Metempsy Technology Consulting
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
 */

#include "mem/cache/prefetch/perceptron_based.hh"

#include <cassert>
#include <climits>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/SignaturePathPrefetcherV2.hh"

namespace Prefetcher {

/**
 * featureTables[0] = Confidence XOR Page addr (4096)
 * featureTables[1] = Cache line (4096)
 * featureTables[2] = Page addr (4096)
 * featureTables[3] = Base addr (4096)
 * featureTables[4] = Confidence (2048)
 * featureTables[5] = Signature XOR Delta (1024)
 * featureTables[6] = PC XOR Delta (128)
 * */

PerceptronBased::PerceptronBased(const SignaturePathPrefetcherParams *p)
        :threshold(p->prefetch_filter_threshold),
        ppf(),
        prefetchTable(p->prefetch_table_assoc, p->prefetch_table_entries,
                    p->prefetch_table_indexing_policy,
                    p->prefetch_table_replacement_policy,
                    PrefetchEntry()),
        rejectTable(p->reject_table_assoc, p->reject_table_entries,
                    p->reject_table_indexing_policy,
                    p->reject_table_replacement_policy,
                    RejectEntry()),
        blkSize(p->block_size),
        pageBytes(0)
        {
                std::cout << "Initialised PPF " << std::endl;
        }

void PerceptronBased::getIndices(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig, std::vector<int> &indices) {
        indices.reserve(ppf.numFeatures);
        indices[0] = ((int)confidence ^ ppn) % 4096;
        indices[1] = ((addr % pageBytes) / blkSize)%4096;
        indices[2] = ppn%4096;
        indices[3] = addr%4096;
        indices[4] = (int)confidence%2048;
        indices[5] = (sig ^ stride)%1024;
        indices[6] = (pc ^ stride)%128;
}

int PerceptronBased::computeWeightSum(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig) {
        int sum = 0;
        std::vector<int> indices;
        getIndices(addr, pc, ppn, stride, confidence, sig, indices);
        for(int i = 0; i < ppf.numFeatures; i++) {
                sum += ppf.featureTables[i].weights[indices[i]];
        }
        return sum; 
}

bool PerceptronBased::infer(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig) {
        
        int result = computeWeightSum(addr, pc, ppn, stride, confidence, sig);
        if (result >= threshold) {
                PrefetchEntry *p = prefetchTable.findVictim(addr);
                prefetchTable.insertEntry(addr, true, p);
        }
        else {
                RejectEntry *p = rejectTable.findVictim(addr);
                rejectTable.insertEntry(addr, true, p);
        }
        return true;
}

void PerceptronBased::train(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig) {



}

} // namespace Prefetcher

