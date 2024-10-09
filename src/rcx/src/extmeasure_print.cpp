
///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2019, Nefelus Inc
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "dbUtil.h"
#include "rcx/extMeasureRC.h"
#include "rcx/extRCap.h"
#include "rcx/extSegment.h"
#include "utl/Logger.h"

#ifdef HI_ACC_1
#define FRINGE_UP_DOWN
#endif
// #define CHECK_SAME_NET
// #define MIN_FOR_LOOPS

namespace rcx
{

    using utl::RCX;
    using namespace odb;
    
    void extMeasure::GetOUname(char buf[20], int met, int metOver, int metUnder)
    {
        if (metUnder > 0 && metOver > 0)
            sprintf(buf, "M%doM%duM%d", met, metUnder, metOver);
        else if (metUnder > 0)
            sprintf(buf, "M%doM%d", met, metUnder);
        else if (metOver > 0)
            sprintf(buf, "M%duM%d", met, metOver);
        else
            sprintf(buf, "M%d", met);
    }
    void extMeasure::PrintCrossSeg(FILE *fp, int x1, int len, int met, int metOver, int metUnder, const char *prefix)
    {
        if (fp==NULL)
            return;
        char buf[20];
        GetOUname(buf,  met,  metOver,  metUnder);
        fprintf(fp, "%s%7.3f %7.3f %7s %dL\n",prefix, GetDBcoords(x1), GetDBcoords(x1+len), buf, len);
    }
    void extMeasure::PrintOUSeg(FILE *fp, int x1, int len, int met, int metOver, int metUnder, const char *prefix, int up_dist, int down_dist)
    {
        if (fp==NULL)
            return;
        char buf[20];
        GetOUname(buf,  met,  metOver,  metUnder);
        fprintf(fp, "%s%7.3f %7.3f %7s %dL up%d down%d\n",prefix, GetDBcoords(x1), GetDBcoords(x1+len), buf, len, up_dist, down_dist);
    }

    void extMeasure::PrintCrossOvelaps(Ath__wire *w, uint tgt_met, int x1, int len, Ath__array1D<extSegment *> *segTable, int totLen, const char *prefix, int metOver, int metUnder)
    {
        if (_segFP!=NULL && segTable->getCnt()>0)
        {
            fprintf(_segFP, "%s %dL cnt %d \n", prefix, totLen, segTable->getCnt());
            PrintCrossSeg(_segFP, x1, len, w->getLevel(), -1, -1, "\t");
            for (uint ii = 0; ii < segTable->getCnt(); ii++)
            {
                extSegment *s = segTable->get(ii);
                PrintCrossSeg(_segFP, s->_ll[!_dir], s->_len, tgt_met, s->_metOver, s->_metUnder, "\t");
            }
        }
    }
    void extMeasure::PrintOverlapSeg(FILE *fp, extSegment *s, int tgt_met, const char *prefix)
    {
        if (fp!=NULL)
            fprintf(fp, "%s%7.3f %7.3f  %dL\n", prefix, GetDBcoords(s->_xy), GetDBcoords(s->_xy + s->_len), s->_len);
    }

    void extMeasure::PrintOvelaps(extSegment *w, uint met, uint tgt_met, Ath__array1D<extSegment *> *segTable, const char *ou)
    {
        if (_segFP != NULL && segTable->getCnt() > 0)
        {
            fprintf(_segFP, "\n%7.3f %7.3f  %dL M%d%sM%d cnt=%d\n", GetDBcoords(w->_xy), GetDBcoords(w->_xy + w->_len), w->_len, met, ou, tgt_met, segTable->getCnt());
            for (uint ii = 0; ii < segTable->getCnt(); ii++)
            {
                extSegment *s = segTable->get(ii);
                PrintOverlapSeg(_segFP, s, tgt_met, "");
            }
        }
    }
    void extMeasure::PrintCrossOvelapsOU(Ath__wire *w, uint tgt_met, int x1, int len, Ath__array1D<extSegment *> *segTable, int totLen, const char *prefix, int metOver, int metUnder)
    {
        if (_segFP != NULL && segTable->getCnt() > 0)
        {
            for (uint ii = 0; ii < segTable->getCnt(); ii++)
            {
                extSegment *s = segTable->get(ii);
                if (s->_metOver == metOver && s->_metUnder == metUnder)
                    PrintCrossSeg(_segFP, s->_xy, s->_len, tgt_met, s->_metOver, s->_metUnder, "\t");
            }
        }
    }
}

