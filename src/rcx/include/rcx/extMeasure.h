
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

#ifndef EXTMEASURE_Y
#define EXTMEASURE_Y

// TO_DELETE #include <darr.h>
#include <dbExtControl.h>
#include <dbShape.h>
#include <dbUtil.h>
#include <gseq.h>
#include "util.h"

// TO_DELETE #include "ISdb.h"
// TO_DELETE #include "ZObject.h"
#include "db.h"
#include "wire.h"
#include "extprocess.h"
#include "gseq.h"
#include "odb.h"
#include "../../odb/src/db/dbHashTable.h"
// #include "extRCap.h"

#include "extSegment.h"
#include "extViaModel.h"

namespace utl {
class Logger;
}
namespace rcx {

using namespace odb;

class extMeasure;
class dbHashTable;
class extDistRC;
class extMetRCTable;
class extMain;

class extGeoVarTable;
class extGeoThickTable;
class extDistRC;
class extDistRCTable;
class extDistWidthRCTable;
class extMetRCTable;
class extRCTable;
class extRCModel;
class extNetStats;
class extLenOU;
class extWire;
class extWireBin;
class extTileSystem;
class extWindow;
class extMainOptions;
class extCorner;
class extMain;

struct ext2dBox;

using utl::Logger;
using namespace odb;

class extMeasure
{

public:
    Ath__gridTable *_search;
    bool _skip_delims;
    extMeasure(utl::Logger* logger);
    // extMeasure();
    ~extMeasure();

// dkf 101052024 ---------------------
uint createContextGrid_dir(char* dirName, const int bboxLL[2], const int bboxUR[2], int met);
// DKF 7/25/2024 -- 3d pattern generation
   int _simVersion;

    int PrintAllGrids(uint dir, FILE *fp, uint mode);

    uint FindSegments(bool lookUp, uint dir, int maxDist, Ath__wire *w1, int xy1, int len1, Ath__wire *w2_next, Ath__array1D<extSegment *> *segTable);

    uint FindSegments_org(bool lookUp, uint dir, int maxDist, Ath__wire *w1, int xy1, int len1, Ath__wire *w2, Ath__array1D<extSegment *> *segTable);
    int GetDx1Dx2(int xy1, int len1, extSegment *w2, int &dx2);
    int GetDx1Dx2(Ath__wire *w1, Ath__wire *w2, int &dx2);
    int GetDx1Dx2(int xy1, int len1, Ath__wire *w2, int &dx2);
    int GetDistance(Ath__wire *w1, Ath__wire *w2);

    // dkf 10162023
    bool OverlapOnly(int xy1, int len1, int xy2, int len2);
    bool Enclosed(int x1, int x2, int y1, int y2);

    // dkf 10082023
    extSegment *_currentSeg;
    bool _newDiagFlow;

    // dkf 10202023
    FILE *_segFP;

    // dkf 10122023
    Ath__array1D<Ath__wire *> **_verticalPowerTable;

    // dkf 10202023

    void PrintCrossSeg(FILE *fp, int x1, int x2, int met, int metOver, int metUnder, const char *prefix = "");
    void GetOUname(char buf[20], int met, int metOver, int metUnder);
    bool GetCrossOvelaps(Ath__wire *w, uint tgt_met, int x1, int x2, uint dir, Ath__array1D<extSegment *> *segTable, Ath__array1D<extSegment *> *whiteTable);
    void PrintCrossOvelaps(Ath__wire *w, uint tgt_met, int x1, int x2, Ath__array1D<extSegment *> *segTable, int totLen, const char *prefix, int metOver = -1, int metUnder = -1);

    // dkf 10212023
    void PrintCrossOvelapsOU(Ath__wire *w, uint tgt_met, int x1, int len, Ath__array1D<extSegment *> *segTable, int totLen, const char *prefix, int metOver, int metUnder);

    // dkf 10232023
    void PrintOverlapSeg(FILE *fp, extSegment *s, int tgt_met, const char *prefix);
    void PrintOvelaps(extSegment *w, uint met, uint tgt_met, Ath__array1D<extSegment *> *segTable, const char *ou);
    void PrintOUSeg(FILE *fp, int x1, int len, int met, int metOver, int metUnder, const char *prefix, int up_dist, int down_dist);

    // dkf 10052023
    void Release(Ath__array1D<extSegment *> *segTable);

    // dkf 10092023
    uint CalcDiag(uint targetMet, uint diagDist, uint tgWidth, uint len1, extSegment *s, int rsegId);

    void FindSegmentsTrack(Ath__wire *w1, int xy1, int len1, Ath__wire *w2_next, uint ii, Ath__array1D<Ath__wire *> *trackTable, bool lookUp, uint dir, int maxDist, Ath__array1D<extSegment *> *segTable);

    static FILE *OpenFile(const char *name, const char *perms);
    FILE *OpenPrintFile(uint dir, const char *name);

    // dkf 09122023
    int SingleDiagTrackDist_opt(SEQ *s, Ath__array1D<SEQ *> *dgContext, bool skipZeroDist, bool skipNegativeDist, Ath__array1D<int> *sortedDistTable, Ath__array1D<SEQ *> *segFilteredTable);
    // dkf 08022023
    int SingleDiagTrackDist(SEQ *s, Ath__array1D<SEQ *> *dgContext, bool skipZeroDist, bool skipNegativeDist, std::vector<int> &distTable, Ath__array1D<SEQ *> *segFilteredTable);
    // dkf 08022023
    int DebugPrint(SEQ *s, Ath__array1D<SEQ *> *dgContext, int trackNum, int plane);
    // dkf 08022023
    uint computeResLoop(Ath__array1D<SEQ *> *tmpTable,
                        Ath__array1D<SEQ *> *dgTable,
                        uint targetMet,
                        uint dir,
                        uint planeIndex,
                        uint trackn,
                        Ath__array1D<SEQ *> *residueSeq);
    bool _no_debug;
    void rcNetInfo();
    bool rcSegInfo();
    bool ouCovered_debug(int covered);
    void segInfo(const char *msg, uint netId, int rsegId);
    bool isVia(uint rsegId);
    bool ouRCvalues(const char *msg, uint jj);
    bool OverSubDebug(extDistRC *rc, int lenOverSub, int lenOverSub_res);
    bool OverSubDebug(extDistRC *rc, int lenOverSub, int lenOverSub_res, double res, double cap, const char *openDist);
    bool Debug_DiagValues(double res, double cap, const char *openDist);
    bool IsDebugNet();
    bool IsDebugNet1();
    bool DebugStart(bool allNets = false);
    bool DebugDiagCoords(int met, int targetMet, int len1, int diagDist, int ll[2], int ur[2], const char * = "");
    double GetDBcoords(uint coord);
    double GetDBcoords(int coord);
    void printNetCaps();

    // dkf 09272023
    FILE *OpenDebugFile();
    bool DebugStart(FILE *fp, bool allNets = false);
    bool DebugEnd(FILE *fp, int OU_covered);
    const char *srcWord(int &rsegId);
    void printNetCaps(FILE *fp, const char *msg);
    void printCaps(FILE *fp, double totCap, double cc, double fr, double dfr, double res, const char *msg);
    void PrintCoords(FILE *fp, int x, int y, const char *xy);
    void PrintCoord(FILE *fp, int x, const char *xy);
    void PrintCoords(FILE *fp, int ll[2], const char *xy);
    bool PrintCurrentCoords(FILE *fp, const char *msg, uint rseg);
    void segInfo(FILE *fp, const char *msg, uint netId, int rsegId);
    double getCC(int rsegId);
    void DebugStart_res(FILE *fp);
    void DebugRes_calc(FILE *fp, const char *msg, int rsegId1, const char *msg_len, uint len, int dist1, int dist2, int tgtMet, double tot, double R, double unit, double prev);
    bool DebugDiagCoords(FILE *fp, int met, int targetMet, int len1, int diagDist, int ll[2], int ur[2], const char *msg);
    void DebugEnd_res(FILE *fp, int rseg1, int len_covered, const char *msg);

    void GetPatName(int met, int overMet, int underMet, char tmp[50]);
    void printDebugRC(FILE *fp, extDistRC *rc, const char *msg, const char *post = "");
    void printDebugRC(FILE *fp, extDistRC *rc, int met, int overMet, int underMet, int width, int dist, int len, const char *msg);
    void printDebugRC_diag(FILE *fp, extDistRC *rc, int met, int overMet, int underMet, int width, int dist, int len, const char *msg);
    // dkf 09292023
    void DebugUpdateValue(FILE *fp, const char *msg, const char *cap_type, int rsegId, double v, double updated_v);
    void DebugPrintNetids(FILE *fp, const char *msg, int rsegId, const char *eol = "\n");
    void DebugUpdateCC(FILE *fp, const char *msg, int rsegId, int rseg2, double v, double tot);
    bool DebugCoords(FILE *fp, int rsegId, int ll[2], int ur[2], const char *msg);
    const char *GetSrcWord(int rsegId);
    void ResetRCs();

    bool updateCoupCap(dbRSeg *rseg1, dbRSeg *rseg2, int jj, double v, const char *dbg_msg);

    void setLogger(Logger *logger) { logger_ = logger; }

    void printTraceNetInfo(const char *msg, uint netId, int rsegId);
    bool printTraceNet(const char *msg, bool init, dbCCSeg *cc = NULL, uint overSub = 0, uint covered = 0);

    bool parse_setLayer(Ath__parser *parser1, uint &layerNum, bool print = false);
    dbNet *createSingleWireNet(char *name, uint level, bool viaFlag, bool debug, bool skipVias, bool skipBterms = false);
    extDistRC *areaCapOverSub(uint modelNum, extMetRCTable *rcModel);

    extDistRC *getUnderLastWidthDistRC(extMetRCTable *rcModel, uint overMet);
    void createCap(int rsegId1, int rsegId2, double *capTable);
    void areaCap(int rsegId1, uint rsegId2, uint len, uint tgtMet);
    bool verticalCap(int rsegId1, uint rsegId2, uint len, uint diagDist, uint tgtWidth, uint tgtMet);
    extDistRC *getVerticalUnderRC(extMetRCTable *rcModel, uint diagDist, uint tgtWidth, uint overMet);
    int readQcap(extMain *extmain, const char *filename, const char *design, const char *capFile, bool skipBterms, dbDatabase *db);
    int readAB(extMain *extMain, const char *filename, const char *design, const char *capFile, bool skipBterms, dbDatabase *db);
    dbRSeg *getRseg(const char *netname, const char *capMsg, const char *tableEntryName);
    int readCapFile(const char *filename, uint &ccCnt);
    bool getFirstShape(dbNet *net, dbShape &s);

    void swap_coords(SEQ *s);
    uint swap_coords(uint initCnt, uint endCnt, Ath__array1D<SEQ *> *resTable);
    uint getOverlapSeq(uint met, SEQ *s, Ath__array1D<SEQ *> *resTable);
    uint getOverlapSeq(uint met, int *ll, int *ur, Ath__array1D<SEQ *> *resTable);

    bool isBtermConnection(dbRSeg *rseg1, dbRSeg *rseg2);
    bool isConnectedToBterm(dbRSeg *rseg1);
    uint defineBox(CoupleOptions &options);
    void printCoords(FILE *fp);
    void printNet(dbRSeg *rseg, uint netId);
    void updateBox(uint w_layout, uint s_layout, int dir = -1);
    void printBox(FILE *fp);
    uint initWS_box(extMainOptions *opt, uint gCnt);
    dbRSeg *getFirstDbRseg(uint netId);

    uint createNetSingleWire(char *dirName, uint idCnt, uint w_layout, uint s_layout, int dir = -1);
    uint createDiagNetSingleWire(char *dirName, uint idCnt, int base, int w_layout, int s_layout, int dir = -1);
    uint createNetSingleWire_cntx(int met, char *dirName, uint idCnt, int d, int ll[2], int ur[2], int s_layout = -1);
    uint createContextNets(char *dirName, int bboxLL[2], int bboxUR[2], int met, double pitchMult);
    uint getPatternExtend();

    uint createContextObstruction(const char *dirName, int x1, int y1, int bboxUR[2], int met, double pitchMult);
    uint createContextGrid(char *dirName, int bboxLL[2], int bboxUR[2], int met, int s_layout = -1);
    

    double getCCfringe(uint lastNode, uint n, uint start, uint end);
    double getCCfringe3D(uint lastNode, uint n, uint start, uint end);

    void updateForBench(extMainOptions *opt, extMain *extMain);
    uint measureOverUnderCap();
    uint measureOverUnderCap_orig(gs *pixelTable, uint **ouPixelTableIndexMap);
    uint getSeqOverOrUnder(Ath__array1D<SEQ *> *seqTable, gs *pixelTable, uint met, Ath__array1D<SEQ *> *resTable);
    uint computeOverOrUnderSeq(Ath__array1D<SEQ *> *seqTable, uint met, Ath__array1D<SEQ *> *resTable, bool over);
    uint computeOverUnder(int *ll, int *ur, Ath__array1D<SEQ *> *resTable);

    void release(Ath__array1D<SEQ *> *seqTable, gs *pixelTable = NULL);
    void addSeq(Ath__array1D<SEQ *> *seqTable, gs *pixelTable);
    void addSeq(int *ll, int *ur, Ath__array1D<SEQ *> *seqTable, gs *pixelTable = NULL);
    SEQ *addSeq(int *ll, int *ur);
    void copySeq(SEQ *t, Ath__array1D<SEQ *> *seqTable, gs *pixelTable);
    void tableCopyP(Ath__array1D<SEQ *> *src, Ath__array1D<SEQ *> *dst);
    void tableCopy(Ath__array1D<SEQ *> *src, Ath__array1D<SEQ *> *dst, gs *pixelTable);

    uint measureDiagFullOU();
    uint ouFlowStep(Ath__array1D<SEQ *> *overTable);
    int underFlowStep(Ath__array1D<SEQ *> *srcTable, Ath__array1D<SEQ *> *overTable);

    // --------------- dkf 09142023
    bool measureRC_res_dist(Ath__array1D<SEQ *> *tmpTable);
    void measureRC_ids_flags(CoupleOptions &options); // dkf 09142023
    void measureRC_091423(CoupleOptions &options);    // dkf 09142023
    void measureRC(CoupleOptions &options);
    float getOverR_weightedFringe(extMetRCTable *rcModel, int dist1, int dist2);
    // dimitri 09172023
    float weightedFringe(extDistRC *rc1, extDistRC *rc2, bool use_weighted = true);
    bool useWeightedAvg(int &dist1, int &dist2, int underMet);
    int computeAndStoreRC_new(dbRSeg *rseg1, dbRSeg *rseg2, int srcCovered);
    //-----------------------------------------------------------

    int computeAndStoreRC(dbRSeg *rseg1, dbRSeg *rseg2, int srcCovered);
    int computeAndStoreRC_720(dbRSeg *rseg1, dbRSeg *rseg2, int srcCovered);

    double ScaleResbyTrack(bool openEnded, double &dist_track);
    void OverSubRC(dbRSeg *rseg1, dbRSeg *rseg2, int ouCovered, int diagCovered, int srcCovered);
    void OverSubRC_dist(dbRSeg *rseg1, dbRSeg *rseg2, int ouCovered, int diagCovered, int srcCovered);
    // dkf 09212023
    void OverSubRC_dist_new(dbRSeg *rseg1, dbRSeg *rseg2, int ouCovered, int diagCovered, int srcCovered);

    void copySeqUsingPool(SEQ *t, Ath__array1D<SEQ *> *seqTable);
    void seq_release(Ath__array1D<SEQ *> *table);
    void calcOU(uint len);
    void calcRC(dbRSeg *rseg1, dbRSeg *rseg2, uint totLenCovered);
    int getMaxDist(int tgtMet, uint modelIndex);
    void calcRes(int rsegId1, uint len, int dist1, int dist2, int tgtMet);
    void calcRes0(double *deltaRes, uint tgtMet, uint len, int dist1 = 0, int dist2 = 0);
    uint computeRes(SEQ* s,
                  uint targetMet,
                  uint dir,
                  uint planeIndex,
                  uint trackn,
                  Ath__array1D<SEQ*>* residueSeq);
    uint computeRes(SEQ *s,
                    Ath__array1D<SEQ *> *,
                    uint targetMet,
                    uint dir,
                    uint planeIndex,
                    uint trackn,
                    Ath__array1D<SEQ *> *residueSeq);
    int computeResDist(SEQ *s,
                       uint trackMin,
                       uint trackMax,
                       uint targetMet,
                       Ath__array1D<SEQ *> *diagTable);
    uint computeDiag(SEQ *s,
                     uint targetMet,
                     uint dir,
                     uint planeIndex,
                     uint trackn,
                     Ath__array1D<SEQ *> *diagTable);

    dbCCSeg *makeCcap(dbRSeg *rseg1, dbRSeg *rseg2, double ccCap);
    void addCCcap(dbCCSeg *ccap, double v, uint model);
    void addFringe(dbRSeg *rseg1, dbRSeg *rseg2, double frCap, uint model);
    void calcDiagRC(int rsegid1, int rsegid2, uint len, uint diagWidth, uint diagDist, uint tgtMet);
    void calcDiagRC(int rsegid1, int rsegid2, uint len, uint dist, uint tgtMet);
    int calcDist(int *ll, int *ur);

    void ccReportProgress();
    uint getOverUnderIndex();

    uint getLength(SEQ *s, int dir);
    uint blackCount(uint start, Ath__array1D<SEQ *> *resTable);
    extDistRC *computeR(uint len, double *valTable);
    extDistRC *computeOverFringe(uint overMet, uint overWidth, uint len, uint dist);
    extDistRC *computeUnderFringe(uint underMet, uint underWidth, uint len, uint dist);

    extDistRC* getDiagUnderCC1(extMetRCTable* rcModel, uint dist, uint overMet) ;

    double getDiagUnderCC(extMetRCTable *rcModel, uint diagWidth, uint diagDist, uint overMet);
      double getDiagUnderCC(extMetRCTable* rcModel, uint dist, uint overMet);

    extDistRC *getDiagUnderCC2(extMetRCTable *rcModel, uint diagWidth, uint diagDist, uint overMet);
    extDistRC *computeOverUnderRC(uint len);
    extDistRC *computeOverRC(uint len);
    extDistRC *computeUnderRC(uint len);
    extDistRC *getOverUnderFringeRC(extMetRCTable *rcModel);
    extDistRC *getOverUnderRC(extMetRCTable *rcModel);
    extDistRC *getOverRC(extMetRCTable *rcModel);
    // DKF 9142023
    float getOverR_weightedFringe(extMetRCTable *rcModel, uint width, int met, int metUnder, int dist1, int dist2);
    float getUnderRC_weightedFringe(extMetRCTable *rcModel, uint width, int met, int metOver, int dist1, int dist2);
    float getOverUnderRC_weightedFringe(extMetRCTable *rcModel, uint width, int met, int underMet, int metOver, int dist1, int dist2);
    extDistRC *getOverRC_Dist(extMetRCTable *rcModel, uint width, int met, int metUnder, int dist, int open = -1);
    extDistRC *getUnderRC_Dist(extMetRCTable *rcModel, uint width, int met, int metOver, int dist, int open = -1);
    extDistRC *getOverUnderRC_Dist(extMetRCTable *rcModel, int width, int met, int underMet, int overMet, int dist, int open = -1);
    static int getMetIndexOverUnder(int met, int mUnder, int mOver, int layerCnt, int maxCnt = 10000);
    // DKF 9202023
    extDistRC *getOverOpenRC_Dist(extMetRCTable *rcModel, uint width, int met, int metUnder, int dist);
    float getOverRC_Open(extMetRCTable *rcModel, uint width, int met, int metUnder, int dist1, int dist2);
    extDistRC *addRC_new(extDistRC *rcUnit, uint len, uint jj, bool addCC);
    float getOURC_Open(extMetRCTable *rcModel, uint width, int met, int metUnder, int metOver, int dist1, int dist2);
    // DKF 9232023
    float getOver_over1(extMetRCTable *rcModel, uint width, int met, int metUnder, int dist1, int dist2, int lenOverSub);
    extDistRC *computeOverUnderRC(extMetRCTable *rcModel, uint len);
    float getOU_over1(extMetRCTable *rcModel, int lenOverSub, int dist1, int dist2);

    uint getUnderIndex();
    uint getUnderIndex(uint overMet);
    extDistRC *getUnderRC(extMetRCTable *rcModel);

    extDistRC *getFringe(uint len, double *valTable);

    void tableCopyP(Ath__array1D<int> *src, Ath__array1D<int> *dst);
    void getMinWidth(dbTech *tech);
    uint measureOverUnderCapCJ();
    uint computeOverUnder(int xy1, int xy2, Ath__array1D<int> *resTable);
    uint computeOUwith2planes(int *ll, int *ur, Ath__array1D<SEQ *> *resTable);
    uint intersectContextArray(int pmin, int pmax, uint met1, uint met2, Ath__array1D<int> *tgtContext);
    uint computeOverOrUnderSeq(Ath__array1D<int> *srcTable, uint met, Ath__array1D<int> *resTable, bool over);
    bool updateLengthAndExit(int &remainder, int &totCovered, int len);
    int compute_Diag_Over_Under(Ath__array1D<SEQ *> *seqTable, Ath__array1D<SEQ *> *resTable);
    int compute_Diag_OverOrUnder(Ath__array1D<SEQ *> *seqTable, bool over, uint met, Ath__array1D<SEQ *> *resTable);
    uint measureUnderOnly(bool diagFlag);
    uint measureOverOnly(bool diagFlag);
    uint measureDiagOU(uint ouLevelLimit, uint diagLevelLimit);

    uint mergeContextArray(Ath__array1D<int> *srcContext, int minS, Ath__array1D<int> *tgtContext);
    uint mergeContextArray(Ath__array1D<int> *srcContext, int minS, int pmin, int pmax, Ath__array1D<int> *tgtContext);
    uint makeMergedContextArray(uint met, int minS);
    uint makeMergedContextArray(uint met);
    uint makeMergedContextArray(int pmin, int pmax, uint met, int minS);
    uint makeMergedContextArray(int pmin, int pmax, uint met);
    uint intersectContextArray(Ath__array1D<int> *s1Context, Ath__array1D<int> *s2Context, int minS1, int minS2, Ath__array1D<int> *tgtContext);

    extDistRC *addRC(extDistRC *rcUnit, uint len, uint jj, double frw = 0.0);

    void setMets(int m, int u, int o);
    void setTargetParams(double w,
                         double s,
                         double r,
                         double t,
                         double h,
                         double w2 = 0.0,
                         double s2 = 0.0);
    void setEffParams(double wTop, double wBot, double teff);
    void addCap();
    void printStats(FILE *fp);
    void printMets(FILE *fp);

    ext2dBox *addNew2dBox(dbNet *net,
                          int *ll,
                          int *ur,
                          uint m,
                          uint d,
                          uint id,
                          bool cntx);
    void clean2dBoxTable(int met, bool cntx);
    uint writeRaphael3D(FILE *fp,
                        int met,
                        bool cntx,
                        double x1,
                        double y1,
                        double th);
    uint writeDiagRaphael3D(FILE *fp,
                            int met,
                            bool cntx,
                            double x1,
                            double y1,
                            double th);
    void writeRaphaelPointXY(FILE *fp, double X, double Y);
    void getBox(int met, bool cntx, int &xlo, int &ylo, int &xhi, int &yhi);
    uint getBoxLength(uint ii, int met, bool cntx);

    int getDgPlaneAndTrackIndex(uint tgt_met, int trackDist, int &loTrack, int &hiTrack);
    int computeDiagOU(SEQ *s, uint targetMet, Ath__array1D<SEQ *> *residueSeq);
    int computeDiagOU(SEQ *s, uint trackMin, uint trackMax, uint targetMet, Ath__array1D<SEQ *> *residueSeq);
    void printDgContext();
    void initTargetSeq();
    void getDgOverlap(CoupleOptions &options);
    void getDgOverlap(SEQ *sseq,
                      uint dir,
                      Ath__array1D<SEQ *> *dgContext,
                      Ath__array1D<SEQ *> *overlapSeq,
                      Ath__array1D<SEQ *> *residueSeq);
    void getDgOverlap_res(SEQ *sseq,
                          uint dir,
                          Ath__array1D<SEQ *> *dgContext,
                          Ath__array1D<SEQ *> *overlapSeq,
                          Ath__array1D<SEQ *> *residueSeq);
    void getDgOverlap_res_bak(SEQ *sseq,
                              uint dir,
                              Ath__array1D<SEQ *> *dgContext,
                              Ath__array1D<SEQ *> *overlapSeq,
                              Ath__array1D<SEQ *> *residueSeq);
    void writeBoxRaphael3D(FILE *fp,
                           ext2dBox *bb,
                           int *base_ll,
                           int *base_ur,
                           double y1,
                           double th,
                           double volt);
    uint getRSeg(dbNet *net, uint shapeId);
    void allocOUpool();
    int get_nm(double n) { return 1000 * (n / _dbunit); };

    int _met;
    int _underMet;
    int _overMet;

    bool _skipResCalc;
    bool _useWeighted;

    int _diagResLen;
    uint _len;
    int _diagResDist;
    int _dist;
    uint _width;
    uint _dir;
    uint _layerCnt;
    uint _wireCnt;

    int _ll[2];
    int _ur[2];
    int _ll_tgt[2];
    int _ur_tgt[2];

    int _tmp_ll[2];
    int _tmp_ur[2];

    uint _dirTable[10000];
    int _minSpaceTable[32];

    int _minWidth;
    int _minSpace;
    int _pitch;

    double _w_m;
    int _w_nm;
    double _s_m;
    int _s_nm;
    double _w2_m;
    int _w2_nm;
    double _s2_m;
    int _s2_nm;

    double _r;
    double _t;
    double _h;

    uint _wIndex;
    uint _sIndex;
    uint _dwIndex;
    uint _dsIndex;
    uint _rIndex;
    uint _pIndex;

    double _topWidth;
    double _botWidth;
    double _teff;
    double _heff;
    double _seff;

    double _topWidthR;
    double _botWidthR;
    double _teffR;
    double _peffR;

    bool _benchFlag;
    bool _varFlag;
    bool _3dFlag;
    bool _over;
    bool _res;
    bool _open;        // wire with no neighbor on one side dkf 09202023
    bool _over1;       // wire with min width one neighbor on one side dkf 09222023
    bool _under1;      // wire with min width one neighbor on one side dkf 09222023
    bool _over_under1; // wire with min width one neighbor on one side dkf 09222023

    bool _overUnder;
    bool _diag;
    bool _verticalDiag;
    bool _plate;
    bool _thickVarFlag;
    bool _metExtFlag;
    uint _diagModel;

    extDistRC *_rc[20];
    extDistRC *_tmpRC;
    bool _rcValid;
    extRCTable *_capTable;
    Ath__array1D<double> _widthTable;
    Ath__array1D<double> _spaceTable;
    Ath__array1D<double> _dataTable;
    Ath__array1D<double> _pTable;
    Ath__array1D<double> _widthTable0;
    Ath__array1D<double> _spaceTable0;
    Ath__array1D<double> _diagSpaceTable0;
    Ath__array1D<double> _diagWidthTable0;

    Ath__array1D<SEQ *> ***_dgContextArray; // array
    uint *_dgContextDepth;                       // not array
    uint *_dgContextPlanes;                      // not array
    uint *_dgContextTracks;                      // not array
    uint *_dgContextBaseLvl;                     // not array
    int *_dgContextLowLvl;                       // not array
    int *_dgContextHiLvl;                        // not array
    uint *_dgContextBaseTrack;                   // array
    int *_dgContextLowTrack;                     // array
    int *_dgContextHiTrack;                      // array
    int **_dgContextTrackBase;                   // array
    FILE *_dgContextFile;
    uint _dgContextCnt;

    uint *_ccContextLength;
    Ath__array1D<int> **_ccContextArray;

    Ath__array1D<ext2dBox *>
        _2dBoxTable[2][20]; // assume 20 layers; 0=main net; 1=context
    AthPool<ext2dBox> *_2dBoxPool;
    uint *_ccMergedContextLength;
    Ath__array1D<int> **_ccMergedContextArray;

    dbBlock *_block;
    dbTech *_tech;
    double _capMatrix[100][100];
    uint _idTable[10000];
    uint _mapTable[10000];
    uint _maxCapNodeCnt;

    extMain *_extMain;
    extRCModel *_currentModel;
    Ath__array1D<extMetRCTable *> _metRCTable;
    uint _minModelIndex;
    uint _maxModelIndex;

    uint _totCCcnt;
    uint _totSmallCCcnt;
    uint _totBigCCcnt;
    uint _totSignalSegCnt;
    uint _totSegCnt;

    double _resFactor;
    bool _resModify;
    double _ccFactor;
    bool _ccModify;
    double _gndcFactor;
    bool _gndcModify;

    gs *_pixelTable;
    uint **_ouPixelTableIndexMap;

    Ath__array1D<SEQ *> *_diagTable;
    Ath__array1D<SEQ *> *_tmpSrcTable;
    Ath__array1D<SEQ *> *_tmpDstTable;
    Ath__array1D<SEQ *> *_tmpTable;
    Ath__array1D<SEQ *> *_underTable;
    Ath__array1D<SEQ *> *_ouTable;
    Ath__array1D<SEQ *> *_overTable;

    int _diagLen;
    uint _netId;
    int _rsegSrcId;
    int _rsegTgtId;
    int _netSrcId;
    int _netTgtId;
    FILE *_debugFP;

    AthPool<SEQ> *_seqPool;

    AthPool<extLenOU> *_lenOUPool;
    Ath__array1D<extLenOU *> *_lenOUtable;

    bool _diagFlow;
    bool _btermThreshold;
    bool _toHi;
    bool _sameNetFlag;

    bool _rotatedGs;

    dbCreateNetUtil _create_net_util;
    int _dbunit;

protected:
    Logger *logger_;
};

}
#endif
