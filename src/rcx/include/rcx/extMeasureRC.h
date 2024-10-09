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

#ifndef ADS_extMeasureRC_H
#define ADS_extMeasureRC_H

#include "extRCap.h"

namespace rcx {

class extMeasureRC : public extMeasure
{
    public:
    // dkf 10012023
  int FindCouplingNeighbors(uint dir, uint couplingDist, uint diag_met_limit);
  int FindCouplingNeighbors_down(uint dir, uint couplingDist, uint diag_met_limit);
  void PrintCoupingNeighbors(FILE *fp, uint upCount, uint downCount);
  void PrintWire(FILE *fp, Ath__wire *w, int level, const char *prefix="", const char *postfix="");
  void Print5wires(FILE *fp, Ath__wire *w, uint level=0);
  Ath__wire* FindOverlap(Ath__wire *w, Ath__wire *first_wire);
  void ResetFirstWires(Ath__grid *netGrid, Ath__array1D<Ath__wire *> *firstWireTable, int tr1, int trCnt, uint limitTrackNum);
  // dkf 10022023
  Ath__wire *FindOverlap(Ath__wire *w, Ath__array1D<Ath__wire *> *firstWireTable, int tr);
  // dkf 10032023
  int FindDiagonalNeighbors(uint dir, uint couplingDist, uint diag_met_limit, uint lookUpLevel, uint limitTrackNum);
  bool IsSegmentOverlap(int x1, int len1, int x2, int len2);
  bool IsOverlap(Ath__wire *w, Ath__wire *w2);
  Ath__wire *GetNextWire(Ath__grid *netGrid, uint tr, Ath__array1D<Ath__wire *> *firstWireTable);
  Ath__wire *FindOverlap(Ath__wire *w, Ath__grid *netGrid, uint tr, Ath__array1D<Ath__wire *> *firstWireTable);
  bool CheckWithNeighbors(Ath__wire *w, Ath__wire *prev);
  Ath__array1D<Ath__wire *> ** allocMarkTable(uint n);
  void DeleteMarkTable(Ath__array1D<Ath__wire *> **tbl, uint n);
  void ResetFirstWires(uint m1, uint m2, uint dir, Ath__array1D<Ath__wire *> **firstWireTable);
  int PrintAllGrids(uint dir, FILE *fp, uint mode);

  // dkf 10042023
  void PrintDiagwires(FILE *fp, Ath__wire *w, uint level);
  int CouplingFlow_new(uint dir, uint couplingDist, uint diag_met_limit);

    // dkf 10052023
    void Print(FILE *fp, Ath__array1D<extSegment *> *segTable, uint d, bool lookUp);
    void Print(FILE *fp, extSegment *s, uint d, bool lookUp);
    // void Release(Ath__array1D<extSegment *> *segTable);

    // dkf 10062023
    bool CheckOrdered(Ath__array1D<extSegment *> *segTable);
    bool measure_RC_new(int met, uint dir, extSegment *up, extSegment *down, int xy1, int len);
    bool measure_init(int met, uint dir, extSegment *up, extSegment *down, int xy1, int len);

    // dkf 10072023
    extSegment *CreateUpDownSegment(Ath__wire *w, Ath__wire *up, int xy1, int len1, Ath__wire *down, Ath__array1D<extSegment *> *segTable, int metOver=-1, int metUnder=-1);
    uint FindUpDownSegments(Ath__array1D<extSegment *> *upTable, Ath__array1D<extSegment *> *downTable, Ath__array1D<extSegment *> *segTable, int metOver=-1, int metUnder=-1);
    extSegment *GetNext(uint ii, int &xy1, int &len1, Ath__array1D<extSegment *> *segTable);
    extSegment *GetNextSegment(uint ii, Ath__array1D<extSegment *> *segTable);
    uint CopySegments(bool up, Ath__array1D<extSegment *> *upTable, uint start, uint end, Ath__array1D<extSegment *> *segTable, int maxDist=1000000000, int metOver=-1, int metUnder=-1);
    void PrintUpDown(FILE *fp, extSegment *s);
    void PrintUpDownNet(FILE *fp, Ath__wire *s, int dist, const char *prefix);
    void PrintUpDown(FILE *fp, Ath__array1D<extSegment *> *segTable);
    void BubbleSort(Ath__array1D<extSegment *> *segTable);
    bool measure_init(extSegment *s);
    bool measure_RC_new(extSegment *s, bool skip_res_calc=false); // dkf 06182024
    // dkf 10082023
    bool measureRC_res_dist_new(extSegment *s);
    bool measureRC_res_init(uint rsegId);
    bool measure_init_cap(extSegment *s, bool up);
    extSegment *_currentSeg;
    bool _newDiagFlow;
    // dkf 10092023
    int ConnectWires(uint dir);
    // uint CalcDiag( uint targetMet, uint diagDist, uint tgWidth, uint len1, extSegment *s, int rsegId);
    // dkf 10102023
    int FindDiagonalNeighbors_down(uint dir, uint couplingDist, uint diag_met_limit, uint lookUpLevel, uint limitTrackNum);
    bool CheckWithNeighbors_below(Ath__wire *w, Ath__wire *prev);
    uint CalcDiagBelow(extSegment *s, Ath__wire *dw);
    // dkf 10112023
    int FindDiagonalNeighbors_vertical_up(uint dir, uint couplingDist, uint diag_met_limit, uint lookUpLevel, uint limitTrackNum, bool skipCheckNeighbors);
    int FindDiagonalNeighbors_vertical_power(uint dir, Ath__wire *w, uint couplingDist, uint diag_met_limit, uint limitTrackNum, Ath__array1D<Ath__wire *> **upWireTable);
    void Print(FILE *fp, Ath__array1D<Ath__wire *> *segTable, const char *msg);
    // dkf 10122023
    Ath__array1D<Ath__wire *> **_verticalPowerTable;

    // dkf 10132023
    // uint FindSegments(bool lookUp, uint dir, int maxDist, Ath__wire *w1, int xy1, int len1, Ath__wire *w2, Ath__array1D<extSegment *> *segTable);

    // dkf 10152023
    Ath__wire *FindDiagonalNeighbors_vertical_up_down(Ath__wire *w, bool &found, uint dir, uint level, uint couplingDist, uint limitTrackNum, Ath__array1D<Ath__wire *> **firstWireTable);
    int FindDiagonalNeighbors_vertical_down(uint dir, uint couplingDist, uint diag_met_limit, uint lookUpLevel, uint limitTrackNum, bool skipCheckNeighbors);

    // dkf 10162023
    Ath__wire* FindOverlap_found(Ath__wire *w, Ath__wire *first_wire, bool &found);
    Ath__wire *SetUpDown(Ath__wire *w2, int next_tr, bool found, Ath__wire *first_wire, Ath__array1D<Ath__wire *> *firstWireTable);
    uint FindAllNeigbors_up(Ath__wire *w, uint start_track, uint dir, uint level, uint couplingDist, uint limitTrackNum, Ath__array1D<Ath__wire *> **firstWireTable, Ath__array1D<Ath__wire *> *resTable);
    Ath__wire *FindOverlapWire(Ath__wire *w, Ath__wire *first_wire);

    // dkf 061824
    int CouplingFlow(uint dir, uint couplingDist, uint diag_met_limit, int totWireCnt, uint &totalWiresExtracted, float &previous_percent_extracted);
    // dkf 10172023
    // dkf 061824 int CouplingFlow(uint dir, uint couplingDist, uint diag_met_limit);


    extSegment *CreateUpDownSegment(bool lookUp, Ath__wire *w, int xy1, int len1, Ath__wire *w2, Ath__array1D<extSegment *> *segTable);
    void FindSegmentsTrack(Ath__wire *w1, int xy1, int len1, Ath__wire *w2_next, uint ii, Ath__array1D<Ath__wire *> *trackTable, bool lookUp, uint dir, int maxDist,  Ath__array1D<extSegment *> *segTable);
    uint FindAllNeigbors_down(Ath__wire *w, int start_track, uint dir, uint level, uint couplingDist, uint limitTrackNum, Ath__array1D<Ath__wire *> **firstWireTable, Ath__array1D<Ath__wire *> *resTable);
    bool PrintInit(FILE *fp, bool dbgOverlaps, Ath__wire *w, int x, int y);
    void PrintTable_coupleWires(FILE *fp1, Ath__wire *w, bool dbgOverlaps, Ath__array1D<Ath__wire *> *UpTable, const char *msg);
    void PrintTable_segments(FILE *fp1,  Ath__wire *w, bool lookUp, bool dbgOverlaps, Ath__array1D<extSegment *> *UpSegTable, const char *msg);
    bool DebugWire(Ath__wire *w, int x, int y, int netId=-1);
    uint CreateCouplingSEgments(Ath__wire *w, Ath__array1D<extSegment *> *segTable, Ath__array1D<extSegment *> *upTable, Ath__array1D<extSegment *> *downTable, bool dbgOverlaps, FILE *fp);
    void PrintTable_wires(FILE *fp, bool dbgOverlaps, uint colCnt, Ath__array1D<Ath__wire *> **verticalPowerTable, const char *msg);

    // dkf 10182023
    Ath__array1D<extSegment *> **_upSegTable;
    Ath__array1D<extSegment *> **_downSegTable;
    Ath__array1D<extSegment *> **allocTable(uint n);
    void DeleteTable(Ath__array1D<extSegment *> **tbl, uint n);
    uint FindAllSegments_up(FILE *fp, Ath__wire *w, bool lookUp, uint start_track, uint dir, uint level, uint maxDist, uint couplingDist, uint limitTrackNum, Ath__array1D<Ath__wire *> **firstWireTable, Ath__array1D<extSegment *> **UpSegTable);
    uint FindAllSegments_vertical(FILE *fp, Ath__wire *w, bool lookUp, uint dir, uint maxDist, Ath__array1D<extSegment *> *aboveTable);

    // dkf 10192023
    dbRSeg* GetRseg(int id);
    bool VerticalCap(uint met, uint tgtMet, int rsegId1, uint rsegId2, uint len, uint width, uint tgtWidth, uint diagDist);
    void VerticalCap(Ath__array1D<extSegment *> *segTable, bool look_up);
    bool DiagCap(FILE *fp, Ath__wire *w, bool lookUp, uint maxDist, uint trackLimitCnt, Ath__array1D<extSegment *> *segTable, bool PowerOnly=false);
    bool DiagCouplingCap(uint met, uint tgtMet, int rsegId1, uint rsegId2, uint len, uint width, uint tgtWidth, uint diagDist);

    // dkf 10202023
    FILE *_segFP;
    Ath__array1D<extSegment *> **_ovSegTable;
    Ath__array1D<extSegment *> **_whiteSegTable;

    // void PrintCrossSeg(FILE *fp, int x1, int x2, int met, int metOver, int metUnder, const char *prefix="");
    //  void GetOUname(char buf[20], int met, int metOver, int metUnder);
    bool GetCrossOvelaps(Ath__wire *w, uint tgt_met, int x1, int x2, uint dir, Ath__array1D<extSegment *> *segTable, Ath__array1D<extSegment *> *whiteTable);
    // void PrintCrossOvelaps(Ath__wire *w, uint tgt_met, int x1, int x2, Ath__array1D<extSegment *> *segTable, int totLen, const char *prefix, int metOver=-1, int metUnder=-1);

    // dkf 10212023
    // void PrintCrossOvelapsOU(Ath__wire *w, uint tgt_met, int x1, int len, Ath__array1D<extSegment *> *segTable, int totLen, const char *prefix, int metOver, int metUnder);

    // dkf 10232023
    // void PrintOverlapSeg(FILE *fp, extSegment *s, int tgt_met, const char *prefix);
    // void PrintOvelaps(extSegment *w, uint met, uint tgt_met, Ath__array1D<extSegment *> *segTable, const char *ou);
    // void PrintOUSeg(FILE *fp, int x1, int len, int met, int metOver, int metUnder, const char *prefix, int up_dist, int down_dist);
    void OverUnder(extSegment *cc, uint met, int overMet, int underMet, Ath__array1D<extSegment *> *segTable, const char *ou);
    void OpenEnded2(extSegment *cc, uint len, int met, int overMet, int underMet, FILE *segFP);
    void OpenEnded1(extSegment *cc, uint len, int met, int overMet, int underMet, FILE *segFP);
    void Model1(extSegment *cc, uint len, int met, int metUnder, int metOver, FILE *segFP);
    void OverUnder(extSegment *cc, uint len, int met, int metUnder, int metOver, FILE *segFP);

    // dkf 10242023
    extDistRC *OverUnderRC(extMetRCTable *rcModel, int open, uint width, int dist, uint len, int met, int metUnder, int metOver, FILE *segFP);
    dbRSeg *GetRSeg(extSegment *cc);
    dbRSeg *GetRSeg(uint rsegId);
    double updateCoupCap(dbRSeg *rseg1, dbRSeg *rseg2, int jj, double v);
    void OverlapDown(int overMet, extSegment *coupSeg, extSegment *overlapSeg, uint dir);

    // dkf 10252023
    int wireOverlap(int X1, int DX, int x1, int dx, int *len1, int *len2, int *len3);
    bool FindDiagonalSegments(extSegment *s, extSegment *ww, Ath__array1D<extSegment *> *segDiagTable, Ath__array1D<extSegment *> *resultTable, bool dbgOverlaps, FILE *fp, bool lookUp, int tgt_met=-1);

    // dkf 10302023
    bool CalcRes(extSegment *s);

    // dkf 11012023
    uint ConnectAllWires(Ath__track *track);

    // dkf 061824
    bool printProgress(uint totalWiresExtracted, uint totWireCnt, float &previous_percent_extracted);

};


}


#endif
