/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2016, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "TAppEncHelper360/TExt360EncGop.h"
#include "TLibCommon/TComPic.h"
#include "TLibEncoder/TEncAnalyze.h"
#include "TLibEncoder/TEncGOP.h"

TExt360EncGop::TExt360EncGop()
{
#if SVIDEO_E2E_METRICS
  m_pcTVideoIOYuvInputFile = NULL;
  m_pcOrgPicYuv = NULL;
  m_pcRecPicYuv = NULL;
  m_iLastFrmPOC = 0;
  m_temporalSubsampleRatio = 1;
  m_pRefGeometry = NULL;
  m_pRecGeometry = NULL;
#endif
}

TExt360EncGop::~TExt360EncGop()
{
#if SVIDEO_E2E_METRICS
  if(m_pRefGeometry)
  {
    delete m_pRefGeometry;
    m_pRefGeometry = NULL;
  }
  if(m_pRecGeometry)
  {
    delete m_pRecGeometry;
    m_pRecGeometry = NULL;
  }
  if(m_pcOrgPicYuv)
  {
    m_pcOrgPicYuv->destroy();
    delete m_pcOrgPicYuv;
  }
  if(m_pcRecPicYuv)
  {
    m_pcRecPicYuv->destroy();
    delete m_pcRecPicYuv;
  }
  m_pcTVideoIOYuvInputFile = NULL;
#endif
}

Void TExt360EncGop::calculatePSNRs(TComPic *pcPic)
{
#if SVIDEO_E2E_METRICS
  readOrigPicYuv(pcPic->getPOC());
  reconstructPicYuv(pcPic->getPicYuvRec());
#endif
#if SVIDEO_SPSNR_NN
  if(getSPSNRMetric()->getSPSNREnabled())
  {
#if SVIDEO_E2E_METRICS
    getSPSNRMetric()->xCalculateSPSNR(getOrigPicYuv(), getRecPicYuv());
#else
    getSPSNRMetric()->xCalculateSPSNR(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
#endif
  }
#if SVIDEO_CODEC_SPSNR_NN
  if(getCodecSPSNRMetric()->getSPSNREnabled())
  {
    getCodecSPSNRMetric()->xCalculateSPSNR(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
  }
#endif
#endif
#if SVIDEO_WSPSNR
  if(getWSPSNRMetric()->getWSPSNREnabled())
  {
    getWSPSNRMetric()->xCalculateWSPSNR(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
  }
#if SVIDEO_WSPSNR_E2E
  if(getE2EWSPSNRMetric()->getWSPSNREnabled())
  {
#if SVIDEO_ERP_PADDING
    getE2EWSPSNRMetric()->setPERPFlag(false);
#endif

#if SVIDEO_E2E_METRICS
    getE2EWSPSNRMetric()->xCalculateE2EWSPSNR(getRecPicYuv(),  getOrigPicYuv());
#else
    getE2EWSPSNRMetric()->xCalculateE2EWSPSNR(pcPic->getPicYuvRec(), pcPic->getPOC());
#endif
  }
#endif
#endif
#if SVIDEO_SPSNR_I
  if(getSPSNRIMetric()->getSPSNRIEnabled())
  {
#if SVIDEO_E2E_METRICS
    getSPSNRIMetric()->xCalculateSPSNRI(getOrigPicYuv(), getRecPicYuv());
#else
    getSPSNRIMetric()->xCalculateSPSNRI(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
#endif
  }
#endif
#if SVIDEO_CPPPSNR
  if(getCPPPSNRMetric()->getCPPPSNREnabled())
  {
#if SVIDEO_E2E_METRICS
    getCPPPSNRMetric()->xCalculateCPPPSNR(getOrigPicYuv(), getRecPicYuv());
#else
    getCPPPSNRMetric()->xCalculateCPPPSNR(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
#endif
  }
#endif
#if SVIDEO_VIEWPORT_PSNR
  if(getViewPortPSNRMetric()->isEnabled())
  {
#if SVIDEO_E2E_METRICS
    getViewPortPSNRMetric()->xCalculatePSNR(pcPic, getOrigPicYuv());
#else
    getViewPortPSNRMetric()->xCalculatePSNR(pcPic);
#endif
  }
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  if(getDynamicViewPortPSNRMetric()->isEnabled())
  {
    getDynamicViewPortPSNRMetric()->xCalculateDynamicViewPSNR(pcPic, getOrigPicYuv());
  }
#endif
#if SVIDEO_CF_SPSNR_NN
  if(getCFSPSNRMetric()->getSPSNREnabled())
  { 
    getCFSPSNRMetric()->xCalculateCFSPSNR(getOrigPicYuv(), pcPic->getPicYuvRec());
  }
#endif
#if SVIDEO_CF_SPSNR_I
  if(getCFSPSNRIMetric()->getSPSNRIEnabled())
  { 
    getCFSPSNRIMetric()->xCalculateSPSNRI(getOrigPicYuv(), pcPic->getPicYuvRec());
  }
#endif
#if SVIDEO_CF_CPPPSNR
  if(getCFCPPPSNRMetric()->getCPPPSNREnabled())
  { 
    getCFCPPPSNRMetric()->xCalculateCPPPSNR(getOrigPicYuv(), pcPic->getPicYuvRec());
  }
#endif
}


Void TExt360EncGop::addResult(TEncAnalyze &encAnalyze)
{
  TExt360EncAnalyze &ext360EncAnalyze=encAnalyze.getExt360Info();

#if SVIDEO_SPSNR_NN
  if(getSPSNRMetric()->getSPSNREnabled())
  {
    ext360EncAnalyze.setSPSNREnabled(true);
    ext360EncAnalyze.addSPSNR(getSPSNRMetric()->getSPSNR());
  }
#if SVIDEO_CODEC_SPSNR_NN
  if(getCodecSPSNRMetric()->getSPSNREnabled())
  {
    ext360EncAnalyze.setCodecSPSNREnabled(true);
    ext360EncAnalyze.addCodecSPSNR(getCodecSPSNRMetric()->getSPSNR());
  }
#endif
#endif
#if SVIDEO_WSPSNR
  if(getWSPSNRMetric()->getWSPSNREnabled())
  {
    ext360EncAnalyze.setWSPSNREnabled(true);
    ext360EncAnalyze.addWSPSNR(getWSPSNRMetric()->getWSPSNR());
  }
#if SVIDEO_WSPSNR_E2E
  if(getE2EWSPSNRMetric()->getWSPSNREnabled())
  {
    ext360EncAnalyze.setE2EWSPSNREnabled(true);
    ext360EncAnalyze.addE2EWSPSNR(getE2EWSPSNRMetric()->getWSPSNR());
  }
#endif
#endif
#if SVIDEO_SPSNR_I
  if(getSPSNRIMetric()->getSPSNRIEnabled())
  {
    ext360EncAnalyze.setSPSNRIEnabled(true);
    ext360EncAnalyze.addSPSNRI(getSPSNRIMetric()->getSPSNRI());
  }
#endif
#if SVIDEO_CPPPSNR
  if(getCPPPSNRMetric()->getCPPPSNREnabled())
  {
    ext360EncAnalyze.setCPPPSNREnabled(true);
    ext360EncAnalyze.addCPPPSNR(getCPPPSNRMetric()->getCPPPSNR());
  }
#endif
#if SVIDEO_VIEWPORT_PSNR
  if(getViewPortPSNRMetric()->isEnabled())
  {
    ext360EncAnalyze.setViewPortPSNREnabled(true);
    ext360EncAnalyze.addViewPortPSNR(getViewPortPSNRMetric()->getPSNR(0));
  }
#endif
#if SVIDEO_CF_SPSNR_NN
  if(getCFSPSNRMetric()->getSPSNREnabled())
  {
    ext360EncAnalyze.setCFSPSNREnabled(true);
    ext360EncAnalyze.addCFSPSNR(getCFSPSNRMetric()->getSPSNR());
  }
#endif
#if SVIDEO_CF_SPSNR_I
  if(getCFSPSNRIMetric()->getSPSNRIEnabled())
  {
    ext360EncAnalyze.setCFSPSNRIEnabled(true);
    ext360EncAnalyze.addCFSPSNRI(getCFSPSNRIMetric()->getSPSNRI());
  }
#endif
#if SVIDEO_CF_CPPPSNR
  if(getCFCPPPSNRMetric()->getCPPPSNREnabled())
  {
    ext360EncAnalyze.setCFCPPPSNREnabled(true);
    ext360EncAnalyze.addCFCPPPSNR(getCFCPPPSNRMetric()->getCPPPSNR());
  }
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  if(getDynamicViewPortPSNRMetric()->isEnabled())
  {
    ext360EncAnalyze.setDynamicViewPortPSNREnabled(true);
    ext360EncAnalyze.addDynamicViewPortPSNR(getDynamicViewPortPSNRMetric()->getPSNR(0));
  }
#endif
}


Void TExt360EncGop::printPerPOCInfo()
{
#if SVIDEO_E2E_METRICS
#if SVIDEO_WSPSNR && SVIDEO_WSPSNR_REPORT_PER_FRAME
  if(getWSPSNRMetric()->getWSPSNREnabled())
  {
    printf(" [Y-WSPSNR %6.4lf dB   U-WSPSNR %6.4lf dB   V-WSPSNR %6.4lf dB]", getWSPSNRMetric()->getWSPSNR()[COMPONENT_Y], getWSPSNRMetric()->getWSPSNR()[COMPONENT_Cb], getWSPSNRMetric()->getWSPSNR()[COMPONENT_Cr] );
  }
#endif
#endif
#if SVIDEO_SPSNR_NN && SVIDEO_SPSNR_NN_REPORT_PER_FRAME
#if SVIDEO_CODEC_SPSNR_NN
  if(getCodecSPSNRMetric()->getSPSNREnabled())
  {
    printf(" [Y-C_SPSNR_NN %6.4lf dB   U-C_SPSNR_NN %6.4lf dB   V-C_SPSNR_NN %6.4lf dB]", getCodecSPSNRMetric()->getSPSNR()[COMPONENT_Y], getCodecSPSNRMetric()->getSPSNR()[COMPONENT_Cb], getCodecSPSNRMetric()->getSPSNR()[COMPONENT_Cr] );
  }
#endif
  if(getSPSNRMetric()->getSPSNREnabled())
  {
#if SVIDEO_E2E_METRICS
    printf(" [Y-E2ESPSNR_NN %6.4lf dB   U-E2ESPSNR_NN %6.4lf dB   V-E2ESPSNR_NN %6.4lf dB]", getSPSNRMetric()->getSPSNR()[COMPONENT_Y], getSPSNRMetric()->getSPSNR()[COMPONENT_Cb], getSPSNRMetric()->getSPSNR()[COMPONENT_Cr] );
#else
    printf(" [Y-SPSNR_NN %6.4lf dB    U-SPSNR_NN %6.4lf dB    V-SPSNR_NN %6.4lf dB]", getSPSNRMetric()->getSPSNR()[COMPONENT_Y], getSPSNRMetric()->getSPSNR()[COMPONENT_Cb], getSPSNRMetric()->getSPSNR()[COMPONENT_Cr] );
#endif
  }
#endif
#if !SVIDEO_E2E_METRICS
#if SVIDEO_WSPSNR && SVIDEO_WSPSNR_REPORT_PER_FRAME
  if(getWSPSNRMetric()->getWSPSNREnabled())
  {
    printf(" [Y-WSPSNR %6.4lf dB   U-WSPSNR %6.4lf dB   V-WSPSNR %6.4lf dB]", getWSPSNRMetric()->getWSPSNR()[COMPONENT_Y], getWSPSNRMetric()->getWSPSNR()[COMPONENT_Cb], getWSPSNRMetric()->getWSPSNR()[COMPONENT_Cr] );
  }
#endif
#endif
#if SVIDEO_SPSNR_I && SVIDEO_SPSNR_I_REPORT_PER_FRAME
  if(getSPSNRIMetric()->getSPSNRIEnabled())
  {
#if SVIDEO_E2E_METRICS
    printf(" [Y-E2ESPSNR_I %6.4lf dB   U-E2ESPSNR_I %6.4lf dB   V-E2ESPSNR_I %6.4lf dB]", getSPSNRIMetric()->getSPSNRI()[COMPONENT_Y], getSPSNRIMetric()->getSPSNRI()[COMPONENT_Cb], getSPSNRIMetric()->getSPSNRI()[COMPONENT_Cr] );
#else
    printf(" [Y-SPSNR_I %6.4lf dB   U-SPSNR_I %6.4lf dB   V-SPSNR_I %6.4lf dB]", getSPSNRIMetric()->getSPSNRI()[COMPONENT_Y], getSPSNRIMetric()->getSPSNRI()[COMPONENT_Cb], getSPSNRIMetric()->getSPSNRI()[COMPONENT_Cr] );
#endif
  }
#endif
#if SVIDEO_CPPPSNR && SVIDEO_CPPPSNR_REPORT_PER_FRAME
  if(getCPPPSNRMetric()->getCPPPSNREnabled())
  {
#if SVIDEO_E2E_METRICS
    printf(" [Y-E2ECPPPSNR %6.4lf dB   U-E2ECPPPSNR %6.4lf dB   V-E2ECPPPSNR %6.4lf dB]", getCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Y], getCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Cb], getCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Cr] );
#else
    printf(" [Y-CPPPSNR %6.4lf dB   U-CPPPSNR %6.4lf dB   V-CPPPSNR %6.4lf dB]", getCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Y], getCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Cb], getCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Cr] );
#endif
  }
#endif
#if SVIDEO_WSPSNR_E2E && SVIDEO_WSPSNR_E2E_REPORT_PER_FRAME
  if(getE2EWSPSNRMetric()->getWSPSNREnabled())
  {
    printf(" [Y-E2EWSPSNR %6.4lf dB   U-E2EWSPSNR %6.4lf dB   V-E2EWSPSNR %6.4lf dB]", getE2EWSPSNRMetric()->getWSPSNR()[COMPONENT_Y], getE2EWSPSNRMetric()->getWSPSNR()[COMPONENT_Cb], getE2EWSPSNRMetric()->getWSPSNR()[COMPONENT_Cr] );
  }
#endif
#if SVIDEO_VIEWPORT_PSNR && SVIDEO_VIEWPORT_PSNR_REPORT_PER_FRAME
  if(getViewPortPSNRMetric()->isEnabled())
  {
    for(Int i=0; i<getViewPortPSNRMetric()->getNumOfViewPorts(); i++)
      printf(" [Y-PSNR_VP%d %6.4lf dB   U-PSNR_VP%d %6.4lf dB   V-PSNR_VP%d %6.4lf dB]", i, getViewPortPSNRMetric()->getPSNR(i)[COMPONENT_Y], i, getViewPortPSNRMetric()->getPSNR(i)[COMPONENT_Cb], i, getViewPortPSNRMetric()->getPSNR(i)[COMPONENT_Cr] );
  }
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR && SVIDEO_DYNAMIC_VIEWPORT_PSNR_REPORT_PER_FRAME
  if(getDynamicViewPortPSNRMetric()->isEnabled())
  {
    for(Int i=0; i<getDynamicViewPortPSNRMetric()->getNumOfViewPorts(); i++)
      printf(" [Y-PSNR_DYN_VP%d %6.4lf dB   U-PSNR_DYN_VP%d %6.4lf dB   V-PSNR_DYN_VP%d %6.4lf dB]", i, getDynamicViewPortPSNRMetric()->getPSNR(i)[COMPONENT_Y], i, getDynamicViewPortPSNRMetric()->getPSNR(i)[COMPONENT_Cb], i, getDynamicViewPortPSNRMetric()->getPSNR(i)[COMPONENT_Cr] );
  }
#endif
#if SVIDEO_CF_SPSNR_NN && SVIDEO_CF_SPSNR_NN_REPORT_PER_FRAME
  if(getCFSPSNRMetric()->getSPSNREnabled())
  {
    printf(" [Y-CFSPSNR_NN %6.4lf dB    U-CFSPSNR_NN %6.4lf dB    V-CFSPSNR_NN %6.4lf dB]", getCFSPSNRMetric()->getSPSNR()[COMPONENT_Y], getCFSPSNRMetric()->getSPSNR()[COMPONENT_Cb], getCFSPSNRMetric()->getSPSNR()[COMPONENT_Cr] );
  }
#endif
#if SVIDEO_CF_SPSNR_I && SVIDEO_CF_SPSNR_I_REPORT_PER_FRAME
  if(getCFSPSNRIMetric()->getSPSNRIEnabled())
  {
    printf(" [Y-CFSPSNR_I %6.4lf dB    U-CFSPSNR_I %6.4lf dB    V-CFSPSNR_I %6.4lf dB]", getCFSPSNRIMetric()->getSPSNRI()[COMPONENT_Y], getCFSPSNRIMetric()->getSPSNRI()[COMPONENT_Cb], getCFSPSNRIMetric()->getSPSNRI()[COMPONENT_Cr] );
  }
#endif
#if SVIDEO_CF_CPPPSNR && SVIDEO_CF_CPPPSNR_REPORT_PER_FRAME
  if(getCFCPPPSNRMetric()->getCPPPSNREnabled())
  {
    printf(" [Y-CFCPPPSNR %6.4lf dB   U-CFCPPPSNR %6.4lf dB   V-CFCPPPSNR %6.4lf dB]", getCFCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Y], getCFCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Cb], getCFCPPPSNRMetric()->getCPPPSNR()[COMPONENT_Cr] );
  }
#endif
}

#if SVIDEO_VIEWPORT_PSNR
// Static member function
Void TExt360EncGop::initViewPortPSNR(TEncGOP &encGop, Int iNumVPs)
{
  encGop.getAnalyzeAllData().getExt360Info().initViewPortPSNR(iNumVPs);
  encGop.getAnalyzeIData().getExt360Info().initViewPortPSNR(iNumVPs);
  encGop.getAnalyzePData().getExt360Info().initViewPortPSNR(iNumVPs);
  encGop.getAnalyzeBData().getExt360Info().initViewPortPSNR(iNumVPs);
}
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
Void TExt360EncGop::initDynamicViewPortPSNR(TEncGOP &encGop, Int iNumVPs)
{
    encGop.getAnalyzeAllData().getExt360Info().initDynamicViewPortPSNR(iNumVPs);
    encGop.getAnalyzeIData().getExt360Info().initDynamicViewPortPSNR(iNumVPs);
    encGop.getAnalyzePData().getExt360Info().initDynamicViewPortPSNR(iNumVPs);
    encGop.getAnalyzeBData().getExt360Info().initDynamicViewPortPSNR(iNumVPs);
}
#endif
#if SVIDEO_E2E_METRICS
Void TExt360EncGop::readOrigPicYuv(Int iPOC)
{
  Int iDeltaFrames = iPOC*m_temporalSubsampleRatio - m_iLastFrmPOC;
  Int aiPad[2]={0,0};
  m_pcTVideoIOYuvInputFile->skipFrames(iDeltaFrames, m_iInputWidth, m_iInputHeight, m_inputChromaFomat);
  m_pcTVideoIOYuvInputFile->read(NULL, m_pcOrgPicYuv, IPCOLOURSPACE_UNCHANGED, aiPad, m_inputChromaFomat, false );
  m_iLastFrmPOC = iPOC*m_temporalSubsampleRatio+1;
}
Void TExt360EncGop::reconstructPicYuv(TComPicYuv *pInPicYuv)
{
  //generate the reconstructed picture in source gemoetry domain;
  if((m_pRecGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRecGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRecGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRecGeometry->compactFramePackConvertYuv(pInPicYuv);
  else
    m_pRecGeometry->convertYuv(pInPicYuv);
#if SVIDEO_ROT_FIX
  m_pRecGeometry->geoConvert(m_pRefGeometry, true);
#else
  m_pRecGeometry->geoConvert(m_pRefGeometry);
#endif
  if((m_pRefGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRefGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRefGeometry->getSVideoInfo()->iCompactFPStructure)
    m_pRefGeometry->compactFramePack(m_pcRecPicYuv);
  else
    m_pRefGeometry->framePack(m_pcRecPicYuv);
}
Void TExt360EncGop::initE2EMetricsCalc(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, TVideoIOYuv& yuvInputFile, ChromaFormat inputChromaFomat, Int iInputWidth, Int iInputHeight, UInt tempSubsampleRatio)
{
    m_pRefGeometry = TGeometry::create(sRefVideoInfo, pInGeoParam);
    m_pRecGeometry = TGeometry::create(sRecVideoInfo, pInGeoParam);

    m_pcTVideoIOYuvInputFile = &yuvInputFile;
    m_iInputWidth = iInputWidth;
    m_iInputHeight = iInputHeight;
    m_inputChromaFomat = inputChromaFomat;
    m_temporalSubsampleRatio = tempSubsampleRatio;

    m_pcOrgPicYuv = new TComPicYuv;
    m_pcOrgPicYuv->createWithoutCUInfo(iInputWidth, iInputHeight, m_inputChromaFomat, true, S_PAD_MAX, S_PAD_MAX);   
    m_pcRecPicYuv = new TComPicYuv;
    m_pcRecPicYuv->createWithoutCUInfo(iInputWidth, iInputHeight, m_inputChromaFomat, true, S_PAD_MAX, S_PAD_MAX);
}
#endif
