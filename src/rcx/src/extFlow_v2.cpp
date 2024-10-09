///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2024, IC BENCH, Dimitris Fotakis
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

#include <map>
#include <vector>

#include "rcx/dbUtil.h"
#include "rcx/extRCap.h"
#include "rcx/extMeasureRC.h"
#include "utl/Logger.h"
#include "wire.h"

namespace rcx {
using namespace odb;

uint extMain::couplingFlow_v2(Rect& extRect, uint ccFlag, extMeasure* m)
{
  uint ccDist = ccFlag;

  uint sigtype = 9;
  uint pwrtype = 11;

  uint pitchTable[32];
  uint widthTable[32];
  for (uint ii = 0; ii < 32; ii++) {
    pitchTable[ii] = 0;
    widthTable[ii] = 0;
  }
  uint dirTable[16];
  int baseX[32];
  int baseY[32];
  uint layerCnt = initSearchForNets(
      baseX, baseY, pitchTable, widthTable, dirTable, extRect, false);

  uint maxPitch = pitchTable[layerCnt - 1];

  layerCnt = (int) layerCnt > _currentModel->getLayerCnt()
                 ? layerCnt
                 : _currentModel->getLayerCnt();
  int ll[2];
  int ur[2];
  ll[0] = extRect.xMin();
  ll[1] = extRect.yMin();
  ur[0] = extRect.xMax();
  ur[1] = extRect.yMax();

  int lo_gs[2];
  int hi_gs[2];
  int lo_sdb[2];
  int hi_sdb[2];

  Ath__overlapAdjust overlapAdj = Z_noAdjust;
  _useDbSdb = true;
  _search->setExtControl(_block,
                         _useDbSdb,
                         (uint) overlapAdj,
                         _CCnoPowerSource,
                         _CCnoPowerTarget,
                         _ccUp,
                         _allNet,
                         _ccContextDepth,
                         _ccContextArray,
                         _ccContextLength,
                         _dgContextArray,
                         &_dgContextDepth,
                         &_dgContextPlanes,
                         &_dgContextTracks,
                         &_dgContextBaseLvl,
                         &_dgContextLowLvl,
                         &_dgContextHiLvl,
                         _dgContextBaseTrack,
                         _dgContextLowTrack,
                         _dgContextHiTrack,
                         _dgContextTrackBase,
                         m->_seqPool);

  _seqPool = m->_seqPool;

  uint maxWidth = 0;
  uint totPowerWireCnt = powerWireCounter(maxWidth);
  uint totWireCnt = signalWireCounter(maxWidth);
  totWireCnt += totPowerWireCnt;

  if (_dbgOption > 0)
    logger_->info(RCX, 43, "{} wires to be extracted", totWireCnt);

  uint minRes[2];
  minRes[1] = pitchTable[1];
  minRes[0] = widthTable[1];

  const uint trackStep = 1000;
  uint step_nm[2];
  step_nm[1] = trackStep * minRes[1];
  step_nm[0] = trackStep * minRes[1];
  if (maxWidth > ccDist * maxPitch) {
    step_nm[1] = ur[1] - ll[1];
    step_nm[0] = ur[0] - ll[0];
  }
  // _use_signal_tables
  Ath__array1D<uint> sdbPowerTable;
  Ath__array1D<uint> tmpNetIdTable(64000);

  uint totalWiresExtracted = 0;
  float previous_percent_extracted = 0.0;
  _search->_no_sub_tracks = _v2;

  int** limitArray;
  limitArray = new int*[layerCnt];
  for (uint jj = 0; jj < layerCnt; jj++) {
    limitArray[jj] = new int[10];
  }

  FILE* bandinfo = nullptr;
  if (_printBandInfo) {
    if (_getBandWire) {
      bandinfo = fopen("bandInfo.getWire", "w");
    } else {
      bandinfo = fopen("bandInfo.extract", "w");
    }
  }
  for (int dir = 1; dir >= 0; dir--) {
    if (_printBandInfo) {
      fprintf(bandinfo, "dir = %d\n", dir);
    }
    if (dir == 0) {
      enableRotatedFlag();
    }

    lo_gs[!dir] = ll[!dir];
    hi_gs[!dir] = ur[!dir];
    lo_sdb[!dir] = ll[!dir];
    hi_sdb[!dir] = ur[!dir];

    int gs_limit = ll[dir];

    _search->initCouplingCapLoops(dir, ccFlag, NULL, m);

    lo_sdb[dir] = ll[dir] - step_nm[dir];
    int hiXY = ll[dir] + step_nm[dir];
    if (hiXY > ur[dir]) {
      hiXY = ur[dir];
    }

    // DELETE uint stepNum = 0;
    for (; hiXY <= ur[dir]; hiXY += step_nm[dir]) {
      if (ur[dir] - hiXY <= (int) step_nm[dir]) {  // dkf FIXME -- delete
        hiXY = ur[dir] + 5 * ccDist * maxPitch;
      }
      if (_v2)
        hiXY = ur[dir] + step_nm[dir] + 5 * ccDist * maxPitch;

      lo_gs[dir] = gs_limit;
      hi_gs[dir] = hiXY;

      fill_gs4(dir,
               ll,
               ur,
               lo_gs,
               hi_gs,
               layerCnt,
               dirTable,
               pitchTable,
               widthTable);

      m->_rotatedGs = getRotatedFlag();
      m->_pixelTable = _geomSeq;

      // add wires onto search such that    loX<=loX<=hiX
      hi_sdb[dir] = hiXY;

      uint processWireCnt = 0;
      processWireCnt += addPowerNets(dir, lo_sdb, hi_sdb, pwrtype);
      processWireCnt += addSignalNets(dir, lo_sdb, hi_sdb, sigtype);

      extMeasureRC* mrc = (extMeasureRC*) m;
      m->_search = m->_extMain->_search;
      if (_dbgOption > 0)
        mrc->PrintAllGrids(dir, m->OpenPrintFile(dir, "wires.org"), 0);
      // m->PrintAllGrids(dir, m->OpenPrintFile(dir, "wires.org"), 0);

      mrc->ConnectWires(dir);

      if (_dbgOption > 0)
        mrc->PrintAllGrids(dir, m->OpenPrintFile(dir, "wires"), 0);

      mrc->FindCouplingNeighbors(dir, 10, 5);
      mrc->CouplingFlow(dir,
                        10,
                        5,
                        totWireCnt,
                        totalWiresExtracted,
                        previous_percent_extracted);
      float tmpCnt = -10;
      mrc->printProgress(totalWiresExtracted, totWireCnt, tmpCnt);
    }
  }
  if (_printBandInfo) {
    fclose(bandinfo);
  }

  delete _geomSeq;
  _geomSeq = nullptr;

  for (uint jj = 0; jj < layerCnt; jj++) {
    delete[] limitArray[jj];
  }
  delete[] limitArray;

  return 0;
}

void extMeasure::OverSubRC(dbRSeg * rseg1,
                             dbRSeg * rseg2,
                             int ouCovered,
                             int diagCovered,
                             int srcCovered)
  {
    int res_lenOverSub = _len - ouCovered;  // 0228
    res_lenOverSub = 0;                     // 0315 -- new calc
    bool SCALING_RES = false;

    double SUB_MULT_CAP
        = 1.0;  // Open ended resitance should account by 1/4 -- 11/15

    double SUB_MULT_RES = 1.0;
    if (SCALING_RES) {
      double dist_track = 0.0;
      SUB_MULT_RES = ScaleResbyTrack(true, dist_track);
      res_lenOverSub = _len;
    }
    int lenOverSub = _len - ouCovered;
    if (lenOverSub < 0)
      lenOverSub = 0;

    bool rvia1 = rseg1 != NULL && isVia(rseg1->getId());

    if (!((lenOverSub > 0) || (res_lenOverSub > 0))) {
      return;
    }

    _underMet = 0;
    for (uint jj = 0; jj < _metRCTable.getCnt(); jj++) {
      extDistRC* rc = _metRCTable.get(jj)->getOverFringeRC(this);
      if (rc == NULL)
        continue;
      double cap = 0;
      if (lenOverSub > 0) {
        cap = SUB_MULT_CAP * rc->getFringe() * lenOverSub;
        _extMain->updateTotalCap(rseg1, cap, jj);
      }
      double res = 0;
      if (!_extMain->_lef_res && !rvia1) {
        if (res_lenOverSub > 0) {
          extDistRC* rc0 = _metRCTable.get(jj)->getOverFringeRC(this, 0);
          extDistRC* rc_last
              = _metRCTable.get(jj)->getOverFringeRC_last(_met, _width);
          double delta0 = rc0->_res - rc_last->_res;
          if (delta0 < 0)
            delta0 = -delta0;
          if (delta0 < 0.000001)
            SUB_MULT_RES = 1.0;
          // if (lenOverSub>0) {
          res = rc->getRes() * res_lenOverSub;
          res *= SUB_MULT_RES;
          _extMain->updateRes(rseg1, res, jj);
        }
      }
      const char* msg = "OverSubRC (No Neighbor)";
      OverSubDebug(rc, lenOverSub, res_lenOverSub, res, cap, msg);
    }
  }

}  // namespace
