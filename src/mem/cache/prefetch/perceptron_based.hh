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

 /**
  * Implementation of the Signature Path Prefetcher
  *
  * References:
  *     Lookahead prefetching with signature path
  *     J Kim, PV Gratz, ALN Reddy
  *     The 2nd Data Prefetching Championship (DPC2)
  * The filter feature described in the paper is not implemented, as it
  * redundant prefetches are dropped by the cache.
  */

#ifndef __MEM_CACHE_PREFETCH_PERCEPTRON_BASED_HH__
#define __MEM_CACHE_PREFETCH_PERCEPTRON_BASED_HH__

#include "base/sat_counter.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "mem/packet.hh"

struct SignaturePathPrefetcherParams;

namespace Prefetcher {

class PerceptronBased
{
  protected:
    /*Signature type*/
    typedef uint16_t signature_t;
    /*Stride (delta) type*/
    typedef int16_t stride_t;
    /*Addr*/
    typedef uint64_t Addr;

    /*Entry in the prefetch table*/
    struct PrefetchEntry : public TaggedEntry {
      /*Physical address*/
      const Addr mPAddr;
      /*PC*/
      const Addr mPC;
      /*Page number*/
      const Addr mPPN;
      /*Delta*/
      const stride_t mStride;
      /*Confidence*/
      const double mConfidence;
      /*Current Signature*/
      const signature_t mCurrentSignature;

      PrefetchEntry() 
        : mPAddr(0), 
          mPC(0), 
          mPPN(0), 
          mStride(0), 
          mConfidence(0), 
          mCurrentSignature(0) {}

      PrefetchEntry(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig) 
        : mPAddr(addr), 
          mPC(pc), 
          mPPN(ppn), 
          mStride(stride), 
          mConfidence(confidence), 
          mCurrentSignature(sig) {}
    };
    AssociativeSet<PrefetchEntry> prefetchTable;
    
    struct RejectEntry : public TaggedEntry {
      /*Physical address*/
      const Addr mPAddr;
      /*PC*/
      const Addr mPC;
      /*Page number*/
      const Addr mPPN;
      /*Delta*/
      const stride_t mStride;
      /*Confidence*/
      const double mConfidence;
      /*Current Signature*/
      const signature_t mCurrentSignature;

      RejectEntry() 
        : mPAddr(0), 
          mPC(0), 
          mPPN(0), 
          mStride(0), 
          mConfidence(0), 
          mCurrentSignature(0) {}

      RejectEntry(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig) 
        : mPAddr(addr), 
          mPC(pc), 
          mPPN(ppn), 
          mStride(stride), 
          mConfidence(confidence), 
          mCurrentSignature(sig) {}
    };
    AssociativeSet<RejectEntry> rejectTable;


    struct FeatureWeights {
      std::vector<SatCounter> weights;

      FeatureWeights() {}

    };

    struct PrefetcherWeightTable {
      std::vector<FeatureWeights> featureTables;
      int numFeatures;

      PrefetcherWeightTable() {
        numFeatures = 7;
        featureTables.reserve(numFeatures); // Using 8 features instead of 9 since prev 3 PCs not available

        /**
         * featureTables[0] = Confidence XOR Page addr
         * featureTables[1] = Cache line
         * featureTables[2] = Page addr
         * featureTables[3] = Base addr
         * featureTables[4] = Confidence
         * featureTables[5] = Signature XOR Delta
         * featureTables[6] = PC XOR Delta
         * */

        // TODO: Reserve space for weights
        for (int i = 0; i < numFeatures; i++) {
          if (i < 4)
            featureTables[i].weights.reserve(4096);
          else if (i == 4)
            featureTables[i].weights.reserve(2048);
          else if (i == 5)
            featureTables[i].weights.reserve(1024);
          else 
            featureTables[i].weights.reserve(128);
        
          for(int j = 0; j < featureTables[i].weights.size(); j++) {
            featureTables[i].weights[i] = SatCounter(5, 1);
          }
        
        }

      }

    };
    PrefetcherWeightTable ppf;

    const uint32_t threshold;
    unsigned blkSize;
    Addr pageBytes;

    int computeWeightSum(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig);

    void getIndices(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig, std::vector<int> &indices);

  public:
    PerceptronBased(const SignaturePathPrefetcherParams* p);
    ~PerceptronBased() = default;

    void setPageBytes(Addr pb) { pageBytes = pb; }

    bool infer(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig);

    void train(Addr addr, Addr pc, Addr ppn, stride_t stride, double confidence, signature_t sig);

};

} // namespace Prefetcher

#endif//__MEM_CACHE_PREFETCH_PERCEPTRON_BASED_HH__
