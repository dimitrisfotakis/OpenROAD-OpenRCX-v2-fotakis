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

#include "rcx/dbUtil.h"
#include "rcx/extRCap.h"
#include "utl/Logger.h"

namespace rcx {

using utl::RCX;

using odb::dbCCSeg;
using odb::dbNet;
using odb::dbRSeg;

void extMeasure::printTraceNetInfo(const char* msg, uint netId, int rsegId)
{
  if (rsegId <= 0) {
    return;
  }
  dbRSeg* rseg = dbRSeg::getRSeg(_block, rsegId);

  uint shapeId = rseg->getTargetCapNode()->getShapeId();

  int x, y;
  rseg->getCoords(x, y);
  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "Trace:C"
             "         {} {}   {}_S{}__{} {}  {:g}\n",
             x,
             y,
             netId,
             shapeId,
             rsegId,
             msg,
             rseg->getCapacitance(0, 1.0));
}

double extMeasure::GetDBcoords(uint coord)
{
  int db_factor = _extMain->_block->getDbUnitsPerMicron();
  return 1.0 * coord / db_factor;
}

double extMeasure::GetDBcoords(int coord)
{
  int db_factor = _extMain->_block->getDbUnitsPerMicron();
  return 1.0 * coord / db_factor;
}

bool extMeasure::IsDebugNet1()
{
  if (_no_debug)
    return false;

  if (!(_extMain->_debug_net_id > 0))
    return false;

  if (_netSrcId == _extMain->_debug_net_id
      || _netTgtId == _extMain->_debug_net_id)
    return true;
  else
    return false;
}
bool extMeasure::IsDebugNet()
{
  if (_no_debug)
    return false;

  if (!(_extMain->_debug_net_id > 0))
    return false;

  if (_netSrcId <= 0 && _netTgtId <= 0)
    return false;

  if (_netSrcId == _extMain->_debug_net_id
      || _netTgtId == _extMain->_debug_net_id)
    return true;
  else
    return false;
}

void extMeasure::printNetCaps()
{
  if (_netId <= 0)
    return;
  dbNet* net = dbNet::getNet(_block, _netId);
  double gndCap = net->getTotalCapacitance(0, false);
  double ccCap = net->getTotalCouplingCap(0);
  double totaCap = gndCap + ccCap;

  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "Trace:"
             "C"
             " netCap  {:g} CC {:g} {:g} R {:g}  {}",
             totaCap,
             ccCap,
             gndCap,
             net->getTotalResistance(),
             net->getConstName());
}

bool extMeasure::printTraceNet(const char* msg,
                               bool init,
                               dbCCSeg* cc,
                               uint overSub,
                               uint covered)
{
  if (!IsDebugNet())
    return false;

  if (init) {
    if (overSub + covered > 0)
      fprintf(_debugFP, "SUB %d GS %d ", overSub, covered);

    if (_netSrcId == _netId)
      printTraceNetInfo("", _netSrcId, _rsegSrcId);
    else
      printTraceNetInfo("", _netTgtId, _rsegTgtId);

    return true;
  }

  fprintf(_debugFP, "%s   ", msg);
  if (overSub + covered > 0)
    fprintf(_debugFP, " L %d SUB %d GS %d ", _len, overSub, covered);

  if (cc != NULL)
    fprintf(_debugFP, " %g ", cc->getCapacitance());

  printTraceNetInfo("", _netSrcId, _rsegSrcId);
  printTraceNetInfo(" ", _netTgtId, _rsegTgtId);
  fprintf(_debugFP, "\n");
  return true;
}

// -----------------------------------------------------------------
void extMeasure::segInfo(const char* msg, uint netId, int rsegId)
{
  if (rsegId <= 0) {
    return;
  }
  dbRSeg* rseg = dbRSeg::getRSeg(_block, rsegId);

  uint shapeId = rseg->getTargetCapNode()->getShapeId();

  const char* wire
      = rseg != NULL
                && extMain::getShapeProperty_rc(rseg->getNet(), rseg->getId())
                       > 0
            ? "Via "
            : "Wire";

  int x, y;
  rseg->getCoords(x, y);
  debugPrint(
      logger_,
      RCX,
      "debug_net",
      1,
      "Trace:C"
      " ------  Total RC Wire Values --- {} \n\t{}  : {}\n\txHi   : {}\n\tyHi  "
      " : "
      "{}\n\tnetId : {}\n\tshapId: {}\n\trsegId: {}\n\tCap   : {:.5f}\n\tRes   "
      ": {:.5f}\n",
      msg,
      wire,
      wire,
      x,
      y,
      netId,
      shapeId,
      rsegId,
      rseg->getCapacitance(0, 1.0),
      rseg->getResistance(0));
}

void extMeasure::rcNetInfo()
{
  if (_netId <= 0)
    return;
  if (!IsDebugNet())
    return;
  dbNet* net = dbNet::getNet(_block, _netId);
  double gndCap = net->getTotalCapacitance(0, false);
  double ccCap = net->getTotalCouplingCap(0);
  double totaCap = gndCap + ccCap;
  debugPrint(
      logger_,
      RCX,
      "debug_net",
      1,
      "Trace:C"
      " ---- Total Net RC Values \n\tNetId  : {}\n\tNetName: {}\n\tCCap   : "
      "{}\n\tGndCap : {}\n\tnetCap : {}\n\tNetRes : {}\n",
      _netId,
      net->getConstName(),
      ccCap,
      gndCap,
      totaCap,
      net->getTotalResistance());
}

bool extMeasure::rcSegInfo()
{
  if (!IsDebugNet())
    return false;

  rcNetInfo();

  if (_netSrcId == _netId)
    segInfo("SRC", _netSrcId, _rsegSrcId);
  else
    segInfo("DST", _netTgtId, _rsegTgtId);

  return true;
}

bool extMeasure::ouCovered_debug(int covered)
{
  if (!IsDebugNet())
    return false;
  int sub = (int) _len - covered;
  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "Trace:C"
             " ----- OverUnder Lengths\n\tLevel : M{}\n\tWidth : {}\n\tDist "
             " : {}\n\tLen   : {}\n\tOU_len: {}\n\tSubLen: {}\n\tDiag  : {}\n",
             _met,
             _width,
             _dist,
             _len,
             covered,
             sub,
             _diag);

  return true;
}

bool extMeasure::isVia(uint rsegId)
{
  dbRSeg* rseg1 = dbRSeg::getRSeg(_block, rsegId);

  bool rvia1
      = rseg1 != NULL
                && extMain::getShapeProperty_rc(rseg1->getNet(), rseg1->getId())
                       > 0
            ? true
            : false;
  return rvia1;
}

bool extMeasure::ouRCvalues(const char* msg, uint jj)
{
  if (!IsDebugNet())
    return false;

  debugPrint(
      logger_,
      RCX,
      "debug_net",
      1,
      "Trace:C"
      " --- OverUnder Cap/Res Values\n\t{}: {}\n\tfrCap :{:g}\n\tcCap  : "
      "{:g}\n\tdgCap : {:g}\n\tRes  : {:g}\n\n",
      msg,
      msg,
      _rc[jj]->_fringe,
      _rc[jj]->_coupling,
      _rc[jj]->_diag,
      _rc[jj]->_res);

  return true;
}

bool extMeasure::OverSubDebug(extDistRC* rc,
                              int lenOverSub,
                              int lenOverSub_res,
                              double res,
                              double cap,
                              const char* openDist)
{
  if (!IsDebugNet())
    return false;

  char buf[100];
  sprintf(buf, "Over SUB %s", openDist);

  rc->printDebugRC(buf, logger_);

  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "Trace:C"
             " ------ Wire Type: {} \n\tLevel : M{}\n\tWidth : {}\n\tLen   : "
             "{}\n\tSubLen: {}\n\tCap   : {:g}\n\tRes    : {:g}\n",
             openDist,
             _met,
             _width,
             _len,
             lenOverSub,
             cap,
             res);

  rcSegInfo();

  return true;
}

bool extMeasure::Debug_DiagValues(double res, double cap, const char* openDist)
{
  if (!IsDebugNet())
    return false;

  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "Trace:C"
             " ------ {} --------- Res:  {:g}\tDiagCap:  {:g}\n",
             openDist,
             res,
             cap);

  return true;
}

bool extMeasure::OverSubDebug(extDistRC* rc, int lenOverSub, int lenOverSub_res)
{
  if (!IsDebugNet())
    return false;

  rc->printDebugRC("Over SUB", logger_);
  rcSegInfo();

  return true;
}

bool extMeasure::DebugStart(bool allNets)
{
  if (!IsDebugNet() && !allNets)
    return false;
  if (_dist < 0) {
    debugPrint(logger_,
               RCX,
               "debug_net",
               1,
               "[BEGIN DEBUGGING] measureRC:RC ----- BEGIN -- M{} Len={} "
               "({:.3f}) MaxDist",
               _met,
               _len,
               GetDBcoords(_len));
  } else {
    debugPrint(logger_,
               RCX,
               "debug_net",
               1,
               "[BEGIN DEBUGGING] measureRC:RC ----- BEGIN -- M{} Dist={} "
               "({:.3f}) Len={} ({:.3f})",
               _met,
               _dist,
               GetDBcoords(_dist),
               _len,
               GetDBcoords(_len));
  }
  uint debugTgtId = _netSrcId == _netId ? _netSrcId : _netTgtId;

  dbNet* net = dbNet::getNet(_block, debugTgtId);
  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "measureRC:C"
             "\tCoupling Segment: Coords \n\tNeighboring Net : {} {}\n\tloX : "
             "{} {:3f} \n\thiX : {} "
             "{:.3f} \n\tloY : {} {:.3f} \n\thiY : {} {:.3f} \n\tDX  : {} "
             "{:3f} \n\tDY  "
             ": {} {:3f}",
             net->getConstName(),
             debugTgtId,
             _ll[0],
             GetDBcoords(_ll[0]),
             _ur[0],
             GetDBcoords(_ur[0]),
             _ll[1],
             GetDBcoords(_ll[1]),
             _ur[1],
             GetDBcoords(_ur[1]),
             _ur[0] - _ll[0],
             GetDBcoords(_ur[0]) - GetDBcoords(_ll[0]),
             _ur[1] - _ll[1],
             GetDBcoords(_ur[1]) - GetDBcoords(_ll[1]));
  return true;
}
void extMeasure::printCaps(FILE *fp, double totCap, double cc, double fr, double dfr, double res, const char *msg)
{
  fprintf(fp, "%s tot %7.3f  CC  %7.3f  fr %7.3f  wfr %7.3f Res %7.3f",
             msg, totCap, cc, fr, dfr, res);
} 
void extMeasure::printNetCaps(FILE *fp, const char *msg)
{
  if (_netId <= 0)
    return;
  dbNet* net = dbNet::getNet(_block, _netId);
  double gndCap = net->getTotalCapacitance(0, false);
  double ccCap = net->getTotalCouplingCap(0);
  double totCap = gndCap + ccCap;
  double res= net->getTotalResistance(0);

  fprintf(fp, "%s netCap", msg);
  printCaps(fp, totCap,  ccCap, gndCap, 0, res, "");
  fprintf(fp, " %s\n",net->getConstName());
} 
const char *extMeasure::srcWord(int &rsegId)
{
  rsegId = _netSrcId;
  if (_netTgtId == _netId)
  {
    rsegId = rsegId;
    return "TGT";
  }
  return "SRC";  
}
const char *extMeasure::GetSrcWord(int rsegId)
{
  if (_rsegTgtId == rsegId)
    return "TGT";
  
  return "SRC";  
}
void extMeasure::PrintCoord(FILE *fp, int x, const char *xy)
{
  fprintf(fp,"%s %5d %7.3f ", xy, x, GetDBcoords(x));
}
void extMeasure::PrintCoords(FILE *fp, int x, int y, const char *xy)
{
  PrintCoord(fp, x, xy);
  PrintCoord(fp, y, "");
}
void extMeasure::PrintCoords(FILE *fp, int ll[2], const char *xy)
{
  PrintCoords(fp, ll[0], ll[1], xy);
}
bool extMeasure::PrintCurrentCoords(FILE *fp, const char *msg, uint rseg)
{
  int dx= _ur[0] - _ll[0];
  int dy= _ur[1] - _ll[1];

  fprintf(fp,"%s", msg);
  PrintCoords(fp, _ll, "x");
  PrintCoords(fp, _ur, "y");
  fprintf(fp," dx %d  dy %d rseg %d\n",dx, dy, rseg);
  return true;
}
FILE *extMeasure::OpenDebugFile()
{
  if (_debugFP == NULL && _netId > 0)
  {
    char buff[100];
    sprintf(buff, "%d.rc", _netId);
    _debugFP = fopen(buff, "w");
    if (_debugFP == NULL)
    {
      // warning(0, "Cannot Open file %s with permissions w", buff);
      exit(0);
    }
  }
  return _debugFP;
}
bool extMeasure::DebugStart(FILE *fp, bool allNets)
{
  if (!IsDebugNet() && !allNets)
    return false;

  // DELETE uint debugTgtId = _netSrcId == _netId ? _netSrcId : _netTgtId;
  dbNet* net1 = dbNet::getNet(_block, _netId);
  int rsegId;
  const char *src= srcWord(rsegId);

  fprintf(fp,"BEGIN met %d dist %d %7.3f  len %d %7.3f %s rseg %d net %d %s\n",
              _met, _dist, GetDBcoords(_dist), _len, GetDBcoords(_len), src, rsegId, _netId, net1->getConstName());
  
  DebugCoords(fp, _rsegSrcId, _ll, _ur, "BEGIN-coords");
  DebugCoords(fp, _rsegTgtId, _ll_tgt, _ur_tgt, "BEGIN-coords");
  
  printNetCaps(fp, "BEGIN");
  PrintCurrentCoords(fp, "BEGIN coords ", rsegId);
  segInfo(fp, "BEGIN", _netId, rsegId);

  return true;
}
bool extMeasure::DebugEnd(FILE *fp, int OU_covered)
{
  if (!IsDebugNet())
    return false;

  // DELETE uint debugTgtId = _netSrcId == _netId ? _netSrcId : _netTgtId;
  dbNet* net1 = dbNet::getNet(_block, _netId);
  int rsegId;
  const char *src= srcWord(rsegId);

  fprintf(fp,"\tOU_covered %d %7.3f    met %d D%d %7.3f  len %d %7.3f %s rseg %d net %d %s\n",
              OU_covered, GetDBcoords(OU_covered), _met, _dist, GetDBcoords(_dist), _len, GetDBcoords(_len), src, rsegId, _netId, net1->getConstName());
  
  segInfo(fp, "END", _netId, rsegId);
  printNetCaps(fp, "END");

  return true;
}
void extMeasure::DebugStart_res(FILE *fp)
{
  fprintf(fp, "\tBEGIN_res ");
  PrintCoords(fp, _ll, "x");
  PrintCoords(fp, _ur, "y");
  dbNet* net1 = dbNet::getNet(_block, _netId);

  fprintf(fp, " M%d  D%d  L%d   N%d N%d %s\n", _met, _dist, _len, _netSrcId, _netTgtId, net1->getConstName());
}
void extMeasure::DebugRes_calc(FILE *fp, const char *msg, int rsegId1, const char *msg_len, uint len, int dist1, int dist2, int tgtMet, double tot, double R, double unit, double prev)
{
  fprintf(fp, "\t\%s tot %g addR %g unit %g prevR %g -- M%d %s %d d1 %d d2 %d rseg %d\n",
      msg, tot, R, unit, prev, tgtMet, msg_len, len, dist1, dist2, rsegId1);
}
bool extMeasure::DebugDiagCoords(FILE *fp, int met, int targetMet,  int len1, int diagDist, int ll[2], int ur[2], const char *msg)
{
  if (!IsDebugNet())
    return false;
  fprintf(fp, "%s M%d M%d L%d dist %d %.3f ", msg, met, targetMet, len1, diagDist, GetDBcoords(diagDist));
  PrintCoords(fp, ll[0], ur[0], " x ");
  PrintCoords(fp, ll[1], ur[1], " y ");
  fprintf(fp, "\n");

  return true;
}
bool extMeasure::DebugCoords(FILE *fp, int rsegId, int ll[2], int ur[2], const char *msg)
{
  if (!IsDebugNet())
    return false;

  uint dx= ur[0]- ll[0];
  uint dy= ur[1]- ll[1];
  fprintf(fp, "%s DX %d  DY %d  ", msg, dx, dy);
  PrintCoords(fp, ll[0], ur[0], " x ");
  PrintCoords(fp, ll[1], ur[1], " y ");
  fprintf(fp, " r%d %s\n", rsegId, GetSrcWord(rsegId));

  return true;
}
void extMeasure::DebugEnd_res(FILE *fp, int rseg1, int len_covered, const char *msg)
{
  dbRSeg* rseg = dbRSeg::getRSeg(_block, rseg1);
  dbNet *net= rseg->getNet();
  double R0= rseg->getResistance();
  double R= net->getTotalResistance();
  // DELETE double res= rseg->getResistance();
  fprintf(fp, "%s len_down %d rsegR %g  netR %g  %d %d %s\n", msg, len_covered, R0, R, rseg1, net->getId(), net->getConstName());
}
double extMeasure::getCC(int rsegId)
{
  dbRSeg* rseg = dbRSeg::getRSeg(_block, rsegId);
  double tot_cc= 0;
  dbCapNode* n = rseg->getTargetCapNode();
  dbSet<dbCCSeg> ccSegs = n->getCCSegs();
  dbSet<dbCCSeg>::iterator ccitr;
  for (ccitr = ccSegs.begin(); ccitr != ccSegs.end(); ++ccitr) {
        dbCCSeg* cc = *ccitr;
        tot_cc += cc->getCapacitance();
  }





/*
  std::vector<dbCCSeg*> ccSegs;
  rseg->getCcSegs(ccSegs);
  for (uint ii= 0; ii<ccSegs.size(); ii++)
  {
    dbCCSeg *c= ccSegs[ii];
    cc += c->getCapacitance();
  }
  */
  return tot_cc;
}


void extMeasure::segInfo(FILE *fp, const char* msg, uint netId, int rsegId)
{
  if (rsegId <= 0) {
    return;
  }
  dbRSeg* rseg = dbRSeg::getRSeg(_block, rsegId);
  uint shapeId = rseg->getTargetCapNode()->getShapeId();
  const char* wire = rseg != NULL && extMain::getShapeProperty_rc(rseg->getNet(), rseg->getId())> 0 ? "Via " : "Wire";
  dbNet* net1 = dbNet::getNet(_block, netId);

  double gndCap= rseg->getCapacitance(0);
  double cc= getCC(rsegId);
  double totCap= gndCap+cc;
  double res= rseg->getResistance();

  int x, y;
  rseg->getCoords(x, y);
  fprintf(fp, "%s %s ", msg, "rseg ");
  printCaps(fp, totCap, cc, gndCap, 0, res, "");
  fprintf(fp, "\n");
    
  fprintf(fp, "%s %s ", msg, wire);
  PrintCoords(fp, x, y, " ");
  fprintf(fp, " sh %d rseg %d net %d %s\n", shapeId, rsegId,  netId, net1->getConstName());
}
void extMeasure::GetPatName(int met, int overMet, int underMet, char tmp[50])
{
  if (overMet > 0 && underMet > 0)
    sprintf(tmp, "M%doM%duM%d ", met, underMet, overMet);
  else if (underMet > 0)
    sprintf(tmp, "M%doM%d ", met, underMet);
  else if (overMet > 0)
    sprintf(tmp, "M%duM%d", met, overMet);
  else
    sprintf(tmp, "M%doM%d", met, 0);
}
void extMeasure::printDebugRC(FILE *fp, extDistRC *rc, const char *msg, const char *post)
{
  fprintf(fp, "%s s %d  CC %g  fr %g wfr %g dg %g  TC %g  R %g %s", msg, rc->_sep, 
      rc->_coupling, rc->_fringe, rc->_fringeW, rc->_diag, rc->_coupling + rc->_fringe + rc->_diag + rc->_fringeW, rc->_res, post);
}
void extMeasure::printDebugRC(FILE *fp, extDistRC *rc, int met, int overMet, int underMet, int width, int dist, int len, const char *msg)
{
  char tmp[50];
  GetPatName(met, overMet, underMet, tmp);
  fprintf(fp, "\t%s L%d %.3f W%d %.3f D%d %.3f ", tmp, len, GetDBcoords(len), width, GetDBcoords(width), dist, GetDBcoords(dist));
  printDebugRC(fp, rc, msg);
  fprintf(fp, "\n");
}
void extMeasure::printDebugRC_diag(FILE *fp, extDistRC *rc, int met,  int overMet, int underMet, int width, int dist, int len, const char *msg)
{
  char tmp[100];
  GetPatName(met, overMet, underMet, tmp);
  fprintf(fp, "\t%s_diag L%d %.3f W%d %.3f D%d %.3f ", tmp, len, GetDBcoords(len), width, GetDBcoords(width), dist, GetDBcoords(dist));
  printDebugRC(fp, rc, msg);
  fprintf(fp, "\n");
}
void extMeasure::DebugPrintNetids(FILE *fp, const char* msg, int rsegId, const char *eol)
{
  if (rsegId <= 0)
    return;
  
  const char *src= rsegId==_rsegSrcId ? "SRC" : "DST";
  dbRSeg* rseg = dbRSeg::getRSeg(_block, rsegId);
  uint shapeId = rseg->getTargetCapNode()->getShapeId();
  // DELETE const char* wire = rseg != NULL && extMain::getShapeProperty_rc(rseg->getNet(), rseg->getId())> 0 ? "Via " : "Wire";
  dbNet* net = rseg->getNet();

  fprintf(fp, "%s%s s%d r%d N%d %s %s", msg, src, shapeId, rsegId, net->getId(), net->getConstName(), eol);
}
void extMeasure::DebugUpdateValue(FILE *fp, const char *msg, const char *cap_type, int rsegId, double v, double tot) 
{
    if (rsegId <= 0)
      return;
  const char *new_val= (tot-v)*100000 > 0 ? "NEW" : "";
  fprintf(fp, "%s %s %.4f tot %.5f %s ", msg, cap_type, v, tot, new_val);
  DebugPrintNetids(fp, "", rsegId);
  fprintf(fp, "\n");
}
void extMeasure::DebugUpdateCC(FILE *fp, const char *msg, int rseg1, int rseg2, double v, double tot) 
{
  DebugUpdateValue(fp, msg, "CC", rseg1, v, tot);
  DebugUpdateValue(fp, msg, "CC", rseg2, v, tot);
}


bool extMeasure::DebugDiagCoords(int met,
                                 int targetMet,
                                 int len1,
                                 int diagDist,
                                 int ll[2],
                                 int ur[2],
                                 const char *msg)
{
  if (!IsDebugNet())
    return false;
  debugPrint(
      logger_,
      RCX,
      "debug_net",
      1,
      "[DIAG_EXT:C]"
      "\t----- Diagonal Coupling Coords {} ----- \n\tDiag: M{} to M{}\n\tDist: {} {:.3f}\n\tLen : {} {:.3f} \
        \n\tloX : {} {:.3f} \n\thiX : {} {:.3f} \n\tloY : {} {:.3f} \n\thiY : {} {:.3f} \
        \n\tDX  : {} {:.3f} \n\tDY  : {} {:.3f} \n",
      msg,
      met,
      targetMet,
      diagDist,
      GetDBcoords(diagDist),
      len1,
      GetDBcoords(len1),
      ll[0],
      GetDBcoords(ll[0]),
      ur[0],
      GetDBcoords(ur[0]),
      ll[1],
      GetDBcoords(ll[1]),
      ur[1],
      GetDBcoords(ur[1]),
      ur[0] - ll[0],
      GetDBcoords(ur[0]) - GetDBcoords(ll[0]),
      ur[1] - ll[1],
      GetDBcoords(ur[1]) - GetDBcoords(ll[1]));
  return true;
}

// -----------------------------------------------------------------
//
// from extRCmodel.cpp

void extDistRC::printDebug(const char* from,
                           const char* name,
                           uint len,
                           uint dist,
                           extDistRC* rcUnit)
{
  if (rcUnit != NULL)
    debugPrint(
        logger_, RCX, "debug_net", 1, "[DistRC:C]\t--\trcUnit is not NULL");

  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "[DistRC:C]"
             "\t{}: {} {:g} {:g} {:g} R {:g} {} {}",
             from,
             name,
             _coupling,
             _fringe,
             _diag,
             _res,
             len,
             dist);
  if (rcUnit == NULL) {
    debugPrint(logger_, RCX, "debug_net", 1, "[DistRC:C]\t--\trcUnit is NULL");
  } else
    rcUnit->printDebugRC("   ", logger_);
}

void extDistRC::printDebugRC(const char* from, Logger* logger)
{
  debugPrint(logger,
             RCX,
             "debug_net",
             1,
             "[DistRC:C]"
             " ---- {}: extRule\n\tDist  : {}\n\tCouple: {:g}\n\tFringe: "
             "{:g}\n\tDiagC "
             ": {:g}\n\ttotCap: {:g}\n\tRes  : {:g}\n",
             from,
             _sep,
             _coupling,
             _fringe,
             _diag,
             _coupling + _fringe + _diag,
             _res);
}

double extDistRC::GetDBcoords(int x, int db_factor)
{
  return 1.0 * x / db_factor;
}

void extDistRC::printDebugRC_diag(int met,
                                  int overMet,
                                  int underMet,
                                  int width,
                                  int dist,
                                  int dbUnit,
                                  Logger* logger)
{
  char tmp[100];

  sprintf(tmp, "M%d diag to M%d", met, overMet);

  char tmp1[100];
  sprintf(tmp1,
          "Width=%d (%.3f) Dist=%d (%.3f) ",
          width,
          GetDBcoords(width, dbUnit),
          dist,
          GetDBcoords(dist, dbUnit));

  debugPrint(logger,
             RCX,
             "debug_net",
             1,
             "[EXTRULE:C]"
             " ----- extRule ----- {} {}\n\t\t\tDist  : {}\n\t\t\tCouple: "
             "{:g}\n\t\t\tFringe: {:g}\n\t\t\tDiagC "
             ": {:g}\n\t\t\ttotCap: {:g}\n\t\t\tRes   : {:g}\n",
             tmp,
             tmp1,
             _sep,
             _coupling,
             _fringe,
             _diag,
             _coupling + _fringe + _diag,
             _res);
}

void extDistRC::printDebugRC(int met,
                             int overMet,
                             int underMet,
                             int width,
                             int dist,
                             int dbUnit,
                             Logger* logger)
{
  char tmp[100];
  if (overMet > 0 && underMet > 0)
    sprintf(tmp, "M%d over M%d under M%d ", met, underMet, overMet);
  else if (underMet > 0)
    sprintf(tmp, "M%d over M%d ", met, underMet);
  else if (overMet > 0)
    sprintf(tmp, "M%d under M%d", met, overMet);

  char tmp1[100];
  sprintf(tmp1,
          "Width=%d (%.3f) Dist=%d (%.3f) ",
          width,
          GetDBcoords(width, dbUnit),
          dist,
          GetDBcoords(dist, dbUnit));
  if (dist < 0)
    sprintf(
        tmp1, "Width=%d (%.3f) MaxDist ", width, GetDBcoords(width, dbUnit));

  debugPrint(logger,
             RCX,
             "debug_net",
             1,
             "[EXTRULE:C]"
             " ----- extRule ----- {} {}\n\t\t\tDist  : {}\n\t\t\tCouple: "
             "{:g}\n\t\t\tFringe: {:g}\n\t\t\tDiagC "
             ": {:g}\n\t\t\ttotCap: {:g}\n\t\t\tRes   : {:g}\n",
             tmp,
             tmp1,
             _sep,
             _coupling,
             _fringe,
             _diag,
             _coupling + _fringe + _diag,
             _res);
}

void extDistRC::printDebugRC_sum(int len, int dbUnit, Logger* logger)
{
  debugPrint(logger,
             RCX,
             "debug_net",
             1,
             "[DistRC:C]"
             " ----- OverUnder Sum ----- Len={} ({:.3f}) \n\t\t\tCouple: "
             "{:.6f}\n\t\t\tFringe: {:.6f}\n\t\t\tDiagC "
             ": {:.6f}\n\t\t\ttotCap: {:.6f}\n\t\t\tRes   : {:.6f}\n",
             len,
             GetDBcoords(len, dbUnit),
             _coupling,
             _fringe,
             _diag,
             _coupling + _fringe + _diag,
             _res);
}

void extDistRC::printDebugRC_values(const char* msg)
{
  
return;
  debugPrint(logger_,
             RCX,
             "debug_net",
             1,
             "[DistRC:C]"
             " ---- {} ------------ \n\t\t\tCouple: {:.6f}\n\t\t\tFringe: "
             "{:.6f}\n\t\t\tDiagC "
             ": {:.6f}\n\t\t\ttotCap: {:.6f}\n\t\t\tRes   : {:.6f}\n",
             msg,
             _coupling,
             _fringe,
             _diag,
             _coupling + _fringe + _diag,
             _res);
}

}  // namespace rcx
