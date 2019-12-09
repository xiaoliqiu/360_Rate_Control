/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TWSPSNRMetricCalc.cpp
    \brief    WSPSNRMetric class
*/

#include "TWSPSNRMetricCalc.h"

#if SVIDEO_WSPSNR

TWSPSNRMetric::TWSPSNRMetric()
: m_bEnabled(false)
, m_fErpWeight_Y(NULL)
, m_fErpWeight_C(NULL)
, m_fCubeWeight_Y(NULL)
, m_fCubeWeight_C(NULL)
, m_fEapWeight_Y(NULL)
, m_fEapWeight_C(NULL)
, m_fOctaWeight_Y(NULL)
, m_fOctaWeight_C(NULL)
, m_fIcoWeight_Y(NULL)
, m_fIcoWeight_C(NULL)
#if SVIDEO_WSPSNR_SSP
, m_fSspWeight_Y(NULL)
, m_fSspWeight_C(NULL)
#endif
#if SVIDEO_ROTATED_SPHERE
, m_fRspWeight_Y(NULL)
, m_fRspWeight_C(NULL)
#endif
#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
, m_fEcpWeight_Y(NULL)
, m_fEcpWeight_C(NULL)
#endif
, m_codingGeoType(0)
, m_iCodingFaceWidth(0)
, m_iCodingFaceHeight(0)
#if SVIDEO_WSPSNR_E2E
#if !SVIDEO_E2E_METRICS
, m_pcTVideoIOYuvInputFile(NULL)
, m_pRefGeometry(NULL)
, m_pRecGeometry(NULL)
, m_pcOrgPicYuv(NULL)
, m_pcRecPicYuv(NULL)
, m_iLastFrmPOC(0)
, m_temporalSubsampleRatio(1)
#endif
#endif
{
  m_dWSPSNR[0] = m_dWSPSNR[1] = m_dWSPSNR[2] = 0;
}

TWSPSNRMetric::~TWSPSNRMetric()
{
  if (m_fErpWeight_Y)
  {
    free(m_fErpWeight_Y);
    m_fErpWeight_Y = NULL;
  }
  if (m_fErpWeight_C)
  {
    free(m_fErpWeight_C);
    m_fErpWeight_C = NULL;
  }
  if (m_fEapWeight_Y)
  {
    free(m_fEapWeight_Y);
    m_fEapWeight_Y = NULL;
  }
  if (m_fEapWeight_C)
  {
    free(m_fEapWeight_C);
    m_fEapWeight_C = NULL;
  }
  if (m_fCubeWeight_Y)
  {
    free(m_fCubeWeight_Y);
    m_fCubeWeight_Y = NULL;
  }
  if (m_fCubeWeight_C)
  {
    free(m_fCubeWeight_C);
    m_fCubeWeight_C = NULL;
  }
  if(m_fOctaWeight_Y)
  {
    free(m_fOctaWeight_Y);
    m_fOctaWeight_Y=NULL;
  }
  if(m_fOctaWeight_C)
  {
    free(m_fOctaWeight_C);
    m_fOctaWeight_C=NULL;
  }
#if SVIDEO_WSPSNR_E2E
#if !SVIDEO_E2E_METRICS
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
#endif
#endif
#if SVIDEO_WSPSNR_SSP
  if(m_fSspWeight_Y)
  {
    free(m_fSspWeight_Y);
    m_fSspWeight_Y=NULL;
  }
  if(m_fSspWeight_C)
  {
    free(m_fSspWeight_C);
    m_fSspWeight_C=NULL;
  }
#endif
#if SVIDEO_ROTATED_SPHERE
  if (m_fRspWeight_Y)
  {
    free(m_fRspWeight_Y);
    m_fRspWeight_Y = NULL;
  }
  if (m_fRspWeight_C)
  {
    free(m_fRspWeight_C);
    m_fRspWeight_C = NULL;
  }
#endif
#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
  if (m_fEcpWeight_Y)
  {
    free(m_fEcpWeight_Y);
    m_fEcpWeight_Y = NULL;
  }
  if (m_fEcpWeight_C)
  {
    free(m_fEcpWeight_C);
    m_fEcpWeight_C = NULL;
  }
#endif
}

Void TWSPSNRMetric::setOutputBitDepth(Int iOutputBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  for(Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_outputBitDepth[i] = iOutputBitDepth[i];
  }
}

Void TWSPSNRMetric::setReferenceBitDepth(Int iReferenceBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  for(Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_referenceBitDepth[i] = iReferenceBitDepth[i];
  }
}

#if SVIDEO_MTK_MODIFIED_COHP1
void getInsideTriangleBoundary(Int *px0, Int *px1, Int y, Int scaleX, Int scaleY, Int iFaceWidth, Int iFaceHeight)
{
  Int x0, x1;

  if(scaleX == 0 && scaleY == 0)
  {
    Int y_L = y;

    if(y_L <= ((iFaceHeight>>1)-1))
    {
      Double d = 1.0*y_L/(iFaceHeight-1)*((iFaceWidth>>1)-2);
      Int  d_i = (Int(d + 1)>>1)<<1;
      x0 = (iFaceWidth>>1) - 2 - d_i;
      x1 = iFaceWidth - 1 - x0;
    }
    else
    {
      Int y_m = iFaceHeight - 1 - y_L;
      Double d = 1.0*y_m/(iFaceHeight-1)*((iFaceWidth>>1)-2);
      Int  d_i = (Int(d + 1)>>1)<<1;
      x0 = d_i;
      x1 = iFaceWidth - 1 - x0;
    }
  }
  else 
  {
    Int y_L = y<<scaleY;
    //Int y_c = y_L>>getComponentScaleY(origchId);

    if(y_L >= (iFaceHeight>>1))
    {
      y_L ++; 
    } 
  
    if(y_L <= ((iFaceHeight>>1)-1))
    {
      Double d = 1.0*y_L/(iFaceHeight-1)*((iFaceWidth>>1)-2);
      Int  d_i = (Int(d + 1)>>1)<<1;
      x0 = (iFaceWidth>>1) - 2 - d_i;
      x1 = iFaceWidth - 1 - x0;
    }
    else
    {
      Int y_m = iFaceHeight - 1 - y_L;
      Double d = 1.0*y_m/(iFaceHeight-1)*((iFaceWidth>>1)-2);
      Int  d_i = (Int(d + 1)>>1)<<1;
      x0 = d_i;
      x1 = iFaceWidth - 1 - x0;
    }
    x0 = ((x0+1)>>1);
    x1 = (x1>>1);
  }

  *px0 = x0, *px1 = x1;
}
#endif

void TWSPSNRMetric::createTable(TComPicYuv* pcPicD, TGeometry *pcCodingGeomtry)
{
  if(!m_bEnabled)
  {
    return;
  }

  SVideoInfo *pCodingSVideoInfo = pcCodingGeomtry->getSVideoInfo();
  Int iFaceWidth = pCodingSVideoInfo->iFaceWidth;
  Int iFaceHeight = pCodingSVideoInfo->iFaceHeight;
  Int iScaleX = pcPicD->getComponentScaleX(COMPONENT_Cb);
  Int iScaleY = pcPicD->getComponentScaleY(COMPONENT_Cb);
  Double dChromaOffset[2] = {0.0, 0.0}; //[0: X; 1: Y];
  if(pcPicD->getChromaFormat() == CHROMA_420)
  {
    dChromaOffset[0] = (m_iChromaSampleLocType == 0 || m_iChromaSampleLocType == 2)? 0 : 0.5;
    dChromaOffset[1] = (m_iChromaSampleLocType == 2 || m_iChromaSampleLocType == 3)? 0 : 0.5;
  }
  if(pcCodingGeomtry->getType()==SVIDEO_EQUIRECT)
  {
    Double fWeightSum_Y=0;
    Double fWeightSum_C=0;
    Int   iWidth = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iHeight = pcPicD->getHeight(COMPONENT_Y) ;
    Int   iWidthC = pcPicD->getWidth(COMPONENT_Cb) ;
    Int   iHeightC = pcPicD->getHeight(COMPONENT_Cb) ;
    m_fErpWeight_Y=(Double*)malloc(iHeight*sizeof(Double));
    m_fErpWeight_C=(Double*)malloc(iHeightC*sizeof(Double));
#if SVIDEO_ERP_PADDING
    if (pcCodingGeomtry->getSVideoInfo()->bPERP)
    {
      iWidth = iFaceWidth;
      iWidthC = (iFaceWidth >> COMPONENT_Cb);
    }
#endif
    for(Int y=0; y< iHeight; y++)
    {
      m_fErpWeight_Y[y]=scos((y-(iHeight/2-0.5))*S_PI/iHeight);
      fWeightSum_Y += m_fErpWeight_Y[y];
    }
    for(Int y=0; y< iHeightC; y++)
    {
      m_fErpWeight_C[y]=scos(((y<<iScaleY)+dChromaOffset[1]+0.5-iHeight/2)*S_PI/iHeight);
      fWeightSum_C += m_fErpWeight_C[y];
    }

    for(Int y=0; y< iHeight; y++)
    {
      m_fErpWeight_Y[y]=m_fErpWeight_Y[y]/fWeightSum_Y/iWidth;
    }
    for(Int y=0; y< iHeightC; y++)
    {
      m_fErpWeight_C[y]=m_fErpWeight_C[y]/fWeightSum_C/(iWidthC);
    }

  }
  else if(pcCodingGeomtry->getType()==SVIDEO_CUBEMAP)
  {
    Double fWeightSum_Y=0;
    Double fWeightSum_C=0;    

    m_fCubeWeight_Y=(Double*)malloc(iFaceHeight * iFaceWidth*sizeof(Double));
    m_fCubeWeight_C=(Double*)malloc((iFaceHeight >> iScaleY) * (iFaceWidth >> iScaleX)*sizeof(Double));
    for(Int y = 0; y < iFaceHeight; y++ )
    {
      for(Int x=0; x < iFaceWidth; x++)
      {
        Int ci, cj, r2;
        Double d2;
        ci= iFaceWidth/2;
        cj= iFaceHeight/2;
        d2 = (x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
        r2 = (iFaceWidth/2)*(iFaceWidth/2);
        Double weight= 1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2)));
        m_fCubeWeight_Y[iFaceWidth*y+x] = weight;
        fWeightSum_Y += weight;
      }
    }
    for(Int y = 0; y < iFaceHeight; y++ )
    {
      for(Int x=0; x<iFaceWidth; x++)
      {
        m_fCubeWeight_Y[iFaceHeight*y+x] = (m_fCubeWeight_Y[iFaceHeight*y+x])/fWeightSum_Y/6.0;
      }
    }

    for(Int y = 0; y < (iFaceHeight>>iScaleY); y++ )
    {
      for(Int x=0; x< (iFaceWidth>>iScaleX); x++)
      {
        Int ci, cj, r2;
        Double d2;
        ci= iFaceWidth/2;
        cj= iFaceHeight/2;
        d2 = (x*(1<<iScaleX)+dChromaOffset[0]+0.5 - ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5 - ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5 -cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5 -cj);
        r2 = (iFaceWidth/2)*(iFaceWidth/2);
        Double weight= 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
        m_fCubeWeight_C[(iFaceWidth>>iScaleX)*y+x]=weight;
        fWeightSum_C += weight;
      }
    }

    for(Int y = 0; y < (iFaceHeight>>iScaleY); y++ )
    {
      for(Int x=0; x< (iFaceWidth>>iScaleX); x++)
      {
        m_fCubeWeight_C[(iFaceWidth>>iScaleX)*y+x]=(m_fCubeWeight_C[(iFaceWidth>>iScaleX)*y+x])/fWeightSum_C/6.0;
      }
    }
  }
#if SVIDEO_ADJUSTED_CUBEMAP
  else if (pcCodingGeomtry->getType() == SVIDEO_ADJUSTEDCUBEMAP)
  {
    Double fWeightSum_Y = 0;
    Double fWeightSum_C = 0;

    m_fCubeWeight_Y = (Double*)malloc(iFaceHeight * iFaceWidth * sizeof(Double));
    m_fCubeWeight_C = (Double*)malloc((iFaceHeight >> iScaleY) * (iFaceWidth >> iScaleX) * sizeof(Double));
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x < iFaceWidth; x++)
      {
        POSType pu = (POSType)((2.0*(x+0.5))/iFaceWidth-1.0);
        POSType pv = (POSType)((2.0*(y+0.5))/iFaceHeight-1.0);
        
        pu = (pu < 0) ? -pu : pu;
        pv = (pv < 0) ? -pv : pv;
#if IMP_WSPSNR_ACP
        Double weight = 1/(4.0*sqrt(0.34 * 0.34 - 0.09 * pu));
        pu = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pu)) / 0.18);     // acp to cmp
        pu = (1.0 + pu)*iFaceWidth/2.0;

        weight *= 1/(4.0*sqrt(0.34 * 0.34 - 0.09 * pv));
        pv = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pv)) / 0.18);     // acp to cmp
        pv = (1.0 + pv)*iFaceHeight/2.0;

        Int ci, cj, r2; Double d2;
        ci= iFaceWidth/2;
        cj= iFaceHeight/2;
        d2 = (pu-ci)*(pu-ci)+(pv-cj)*(pv-cj);
        r2 = (iFaceWidth/2)*(iFaceWidth/2);
        weight *= (1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2))));
#else
        Double tx = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pu)) / 0.18);
        Double ty = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pv)) / 0.18);
        Double weight = 1.0 / (ssqrt(1.36 * 1.36 - 1.44 * tx) * ssqrt(1.36 * 1.36 - 1.44 * ty) * (tx*tx+ty*ty+1.0) * ssqrt(tx*tx+ty*ty+1.0));
#endif
        m_fCubeWeight_Y[iFaceWidth*y+x] = weight;
        fWeightSum_Y += weight;
      }
    }
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x<iFaceWidth; x++)
      {
        m_fCubeWeight_Y[iFaceHeight*y + x] = (m_fCubeWeight_Y[iFaceHeight*y + x]) / fWeightSum_Y / 6.0;
      }
    }
    for (Int y = 0; y < (iFaceHeight >> iScaleY); y++)
    {
      for (Int x = 0; x< (iFaceWidth >> iScaleX); x++)
      {
        POSType pu = (POSType)((2.0*(x+0.5))/(iFaceWidth>>iScaleX)-1.0);
        POSType pv = (POSType)((2.0*(y+0.5))/(iFaceHeight>>iScaleY)-1.0);

        pu = (pu < 0) ? -pu : pu;
        pv = (pv < 0) ? -pv : pv;
#if IMP_WSPSNR_ACP
        Double weight = 1/(4.0*sqrt(0.34 * 0.34 - 0.09 * pu));
        pu = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pu)) / 0.18);     // acp to cmp
        pu = (1.0 + pu)*iFaceWidth/2.0;

        weight *= 1/(4.0*sqrt(0.34 * 0.34 - 0.09 * pv));
        pv = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pv)) / 0.18);     // acp to cmp
        pv = (1.0 + pv)*iFaceHeight/2.0;

        Int ci, cj, r2; Double d2;
        ci= iFaceWidth/2;
        cj= iFaceHeight/2;
        d2 = (pu-ci)*(pu-ci)+(pv-cj)*(pv-cj);
        r2 = (iFaceWidth/2)*(iFaceWidth/2);
        weight *= (1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2))));
#else
        Double tx = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pu)) / 0.18);
        Double ty = ((0.34-sqrt(0.34 * 0.34 - 0.09 * pv)) / 0.18);

        Double weight = 1.0 / (ssqrt(1.36 * 1.36 - 1.44 * tx) * ssqrt(1.36 * 1.36 - 1.44 * ty) * (tx*tx+ty*ty+1.0) * ssqrt(tx*tx+ty*ty+1.0));
#endif
        m_fCubeWeight_C[(iFaceWidth>>iScaleX)*y+x]=weight;
        fWeightSum_C += weight;
      }
    }

    for (Int y = 0; y < (iFaceHeight >> iScaleY); y++)
    {
      for (Int x = 0; x< (iFaceWidth >> iScaleX); x++)
      {
        m_fCubeWeight_C[(iFaceWidth >> iScaleX)*y + x] = (m_fCubeWeight_C[(iFaceWidth >> iScaleX)*y + x]) / fWeightSum_C / 6.0;
      }
    }
  }
#endif
#if SVIDEO_ADJUSTED_EQUALAREA
  else if(pcCodingGeomtry->getType()==SVIDEO_ADJUSTEDEQUALAREA)
  {
    Int   iWidth = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iHeight = pcPicD->getHeight(COMPONENT_Y) ;
    Int   iWidthC = pcPicD->getWidth(COMPONENT_Cb) ;
    Int   iHeightC = pcPicD->getHeight(COMPONENT_Cb) ;
    Double pv;
    Double zeta;
    Double BETA = 1.0/1.4;

    Double weightSum_Y=0;
    Double weightSum_C=0;
    m_fEapWeight_Y = (Double*)malloc(iWidth*iHeight*sizeof(Double));
    m_fEapWeight_C = (Double*)malloc(iWidthC*iHeightC*sizeof(Double));
    for (Int y=0;y<iHeight;y++)
    {
      pv   = ((Double)y+0.5)/(Double)iHeight;
      zeta = sasin((1.0-2.0*pv)*sin(0.5*S_PI*BETA))/BETA;
      for (Int x=0;x<iWidth;x++)
      { 
        m_fEapWeight_Y[y*iWidth+x] = scos(zeta)/scos(BETA*zeta);
        weightSum_Y               +=  m_fEapWeight_Y[y*iWidth+x];
      }
    }
    for (Int y=0; y<iHeightC; y++)
    {
      pv   = ((Double)y+0.5)/(Double)iHeightC;
      zeta = sasin((1.0-2.0*pv)*sin(0.5*S_PI*BETA))/BETA;
      for (Int x=0; x<iWidthC; x++)
      {
        m_fEapWeight_C[y*iWidthC+x] = scos(zeta)/scos(BETA*zeta);
        weightSum_C               +=  m_fEapWeight_C[y*iWidthC+x];
      }
    }

  // weights normalization
    for (Int y=0;y<iHeight;y++)
    {
      for (Int x=0;x<iWidth;x++)
      {    
          m_fEapWeight_Y[y*iWidth+x] /= weightSum_Y;
      }
    }
    for (Int y=0; y<iHeightC; y++)
    {
      for (Int x=0; x<iWidthC; x++)
      {
        m_fEapWeight_C[y*iWidthC+x] /= weightSum_C;
      }
    }
  }
#else
  else if(pcCodingGeomtry->getType()==SVIDEO_EQUALAREA)
  {
    Int   iWidth = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iHeight = pcPicD->getHeight(COMPONENT_Y) ;
    Int   iWidthC = pcPicD->getWidth(COMPONENT_Cb) ;
    Int   iHeightC = pcPicD->getHeight(COMPONENT_Cb) ;
    m_fEapWeight_Y = (Double*)malloc(iWidth*iHeight*sizeof(Double));
    m_fEapWeight_C = (Double*)malloc(iWidthC*iHeightC*sizeof(Double));
    for (Int y=0;y<iHeight;y++)
    {
      for (Int x=0;x<iWidth;x++)
      {
        m_fEapWeight_Y[y*iWidth+x] = 1.0/(iWidth*iHeight);
      }
    }
    for (Int y=0; y<iHeightC; y++)
    {
      for (Int x=0; x<iWidthC; x++)
      {
        m_fEapWeight_C[y*iWidthC+x]=1.0/(iWidthC*iHeightC);
      }
    }
  }
#endif
  else if(pcCodingGeomtry->getType()==SVIDEO_OCTAHEDRON && pCodingSVideoInfo->iCompactFPStructure ==0 )
  {
    Int iwidth=iFaceWidth*pCodingSVideoInfo->framePackStruct.cols;
    Int iheight=iFaceHeight*pCodingSVideoInfo->framePackStruct.rows;
    Double* fWeightRotZero_Y = (Double*)malloc(iFaceWidth*iFaceHeight*sizeof(Double));
    Double* fWeightRotZero_C = (Double*)malloc((iFaceWidth>>iScaleX) *(iFaceHeight>>iScaleY) *sizeof(Double));
    m_fOctaWeight_Y=(Double*)malloc(iwidth*iheight*sizeof(Double));
    m_fOctaWeight_C=(Double*)malloc((iwidth>>iScaleX)*(iheight>>iScaleY)*sizeof(Double));
    Double weightSum_Y=0;
    Double weightSum_C=0;
    for (Int x=0; x<iFaceWidth; x++)
    {
      for (Int y=0; y<iFaceHeight; y++)
      {
        if(sfabs(x+0.5-iFaceWidth/2) < (y+0.5)/ssqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x] = weight;
          weightSum_Y += weight;
        }
        else
        {
          fWeightRotZero_Y[iFaceWidth*y+x]=0;
        }
      }
    }

    //Normalize per face_Y
    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[iFaceWidth*y+x]=fWeightRotZero_Y[iFaceWidth*y+x]/weightSum_Y/8.0;
      }
    }

  //weights for the entire frame_Y
    Double weight;
    for (Int rows=0;rows<pCodingSVideoInfo->framePackStruct.rows;rows++)
    {
      Int ypos=rows*iFaceHeight;
      for (Int cols=0;cols<pCodingSVideoInfo->framePackStruct.cols;cols++)
      {
        Int xpos=cols*iFaceWidth;
        for (Int x=0;x<iFaceWidth;x++)
        {
          for (Int y=0;y<iFaceHeight;y++)
          {
            if(pCodingSVideoInfo->framePackStruct.faces[rows][cols].rot==0)
            {
              weight=fWeightRotZero_Y[y*iFaceWidth+x];
            }
            else
            {
              weight=fWeightRotZero_Y[(iFaceHeight-y-1)*iFaceWidth + (iFaceWidth-x-1)];
            }
            m_fOctaWeight_Y[(ypos+y)*iwidth+xpos+x]=weight;
          }
        }
      }
    }
   free(fWeightRotZero_Y);

   for (Int x=0; x<(iFaceWidth>>iScaleX); x++)
   {
     for (Int y=0; y<(iFaceHeight>>iScaleY); y++)
     {
       if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (y*(1<<iScaleY)+dChromaOffset[1]+0.5)/ssqrt(3.0))
       {
         Int ci=0;
         Int cj=0;
         Double r2,d2;
         ci = iFaceWidth/2;
         cj = 2*iFaceHeight/3;
         d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
         r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
         Double dweight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
         fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=dweight;
         weightSum_C+=dweight;
       }
       else
       {
         fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=0;
       }
     }
   }

   //Normalize per face_C
   for (Int x=0; x<(iFaceWidth>>iScaleX); x++)
   {
     for (Int y=0; y<(iFaceHeight>>iScaleY); y++)
     {
       fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]/weightSum_C/8.0;
     }
   }
   //weights for the entire frame_C
   for (Int rows=0;rows<pCodingSVideoInfo->framePackStruct.rows;rows++)
   {
     Int ypos=rows*(iFaceHeight>>iScaleY);
     for (Int cols=0; cols<pCodingSVideoInfo->framePackStruct.cols; cols++)
     {
       Int xpos=cols*(iFaceWidth>>iScaleX);
       for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
       {
         for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
         {
           if(pCodingSVideoInfo->framePackStruct.faces[rows][cols].rot==0)
           {
             weight=fWeightRotZero_C[y*(iFaceWidth>>iScaleX)+x];
           }
           else
           {
             weight=fWeightRotZero_C[((iFaceHeight>>iScaleY)-y-1)*(iFaceWidth>>iScaleX)+((iFaceWidth>>iScaleX)-x-1)];
           }
           m_fOctaWeight_C[(ypos+y)*(iwidth>>iScaleX)+xpos+x]=weight;
         }
       }
     }
   }
   free(fWeightRotZero_C);
  }
  else if(pcCodingGeomtry->getType()==SVIDEO_OCTAHEDRON && pCodingSVideoInfo->iCompactFPStructure ==1)
#if SVIDEO_MTK_MODIFIED_COHP1
  {
    Int   iwidth  = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iheight = pcPicD->getHeight(COMPONENT_Y) ;
    Double* fWeightRotZero_Y=(Double*)malloc(iFaceWidth*iFaceHeight*sizeof(Double));
    Double* fWeightRotZero_C=(Double*)malloc((iFaceWidth>>iScaleX)*(iFaceHeight>>iScaleY)*sizeof(Double));
    m_fOctaWeight_Y=(Double*)malloc(iwidth*iheight*sizeof(Double));
    m_fOctaWeight_C=(Double*)malloc((iwidth>>iScaleX)*(iheight>>iScaleY)*sizeof(Double));
    Double weightSum_Y=0;
    Double weightSum_C=0;
#if SVIDEO_COHP1_PADDING
    for(Int x = 0; x < iwidth; x++)
      for(Int y = 0; y < iheight; y++)
        m_fOctaWeight_Y[y*iwidth+x] = 0;

    for(Int x = 0; x < (iwidth>>iScaleX); x++)
      for(Int y = 0; y < (iheight>>iScaleY); y++)
        m_fOctaWeight_C[y*(iwidth>>iScaleX)+x] = 0;

    for(Int x = 0; x < iFaceWidth; x++)
    {
      for(Int y = 0; y < iFaceHeight; y++)
      {
        if(pcCodingGeomtry->insideFace(0, x, y, COMPONENT_Y, COMPONENT_Y))
        {
          Int ci = 0;
          Int cj = 0;
          Double r2, d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 = (x+0.5-ci) * (x+0.5-ci) + (y+0.5-cj) * (y+0.5-cj);
          r2 = (iFaceWidth/ssqrt(6.0)) * (iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2) * sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x] = weight;
          weightSum_Y += weight;
        }
        else
        {
          fWeightRotZero_Y[iFaceWidth*y+x] = 0;
        }
      }
    }

    //Normalize per face_Y
    for(Int x = 0; x < iFaceWidth; x++)
      for(Int y = 0; y < iFaceHeight; y++)
        fWeightRotZero_Y[iFaceWidth*y+x] = fWeightRotZero_Y[iFaceWidth*y+x] / weightSum_Y / 8.0;

    //weights for the entire frame_Y
    for(Int faceIdx = 0; faceIdx < 8; faceIdx++)
    {
      for(Int v = 0; v < iFaceHeight; v++)
        for(Int u = 0; u < iFaceWidth; u++)
          if(pcCodingGeomtry->insideFace(faceIdx, u, v, COMPONENT_Y, COMPONENT_Y))
          {
            IPos tmpPos;
            tmpPos.faceIdx = faceIdx;
            tmpPos.u = u;
            tmpPos.v = v;
            IPos2D framePackedPos;
            pcCodingGeomtry->geoToFramePack(&tmpPos, &framePackedPos);    
            m_fOctaWeight_Y[framePackedPos.y*iwidth + framePackedPos.x] = fWeightRotZero_Y[v*iFaceWidth+u];
          }
    }
    free(fWeightRotZero_Y);

    Int iScaledFaceWidth  = iFaceWidth >> iScaleX;
    Int iScaledFaceHeight = iFaceHeight >> iScaleY;

    for(Int x = 0; x < iScaledFaceWidth; x++)
    {
      for(Int y = 0; y < iScaledFaceHeight; y++)
      {
        if(pcCodingGeomtry->insideFace(0, (x<<iScaleX), (y<<iScaleY), COMPONENT_Y, COMPONENT_Cb))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2)));
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=weight;
          weightSum_C+=weight;
        }
        else
        {
          fWeightRotZero_C[iScaledFaceWidth*y+x]=0;
        }
      }
    }

    //Normalize per face_C
    for (Int x = 0; x < iScaledFaceWidth; x++)
      for (Int y = 0; y < iScaledFaceHeight; y++)
        fWeightRotZero_C[iScaledFaceWidth*y+x] = fWeightRotZero_C[iScaledFaceWidth*y+x] / weightSum_C / 8.0;

    //weights for the entire frame_C
    for(Int faceIdx = 0; faceIdx < 8; faceIdx++)
    {
      for(Int v = 0; v < iScaledFaceHeight; v++)
        for(Int u = 0; u < iScaledFaceWidth; u++)
          if(pcCodingGeomtry->insideFace(faceIdx, (u<<iScaleX), (v<<iScaleY), COMPONENT_Y, COMPONENT_Cb))
          {
            IPos tmpPos;
            tmpPos.faceIdx = faceIdx;
            tmpPos.u = u<<iScaleX;
            tmpPos.v = v<<iScaleY;
            IPos2D framePackedPos;
            pcCodingGeomtry->geoToFramePack(&tmpPos, &framePackedPos);     
            m_fOctaWeight_C[(framePackedPos.y>>iScaleY)*(iwidth>>iScaleY) + (framePackedPos.x>>iScaleX)] = fWeightRotZero_C[v*(iFaceWidth>>iScaleX)+u];
          }
    }
    free(fWeightRotZero_C);
#else
    Int i, x, y;

    for (x=0;x<iwidth;x++)
    {
      for (y=0;y<iheight;y++)
      {
        m_fOctaWeight_Y[y*iwidth+x]=-1;
      }
    }

    for (x=0;x<(iwidth>>iScaleX);x++)
    {
      for (y=0;y<(iheight>>iScaleY);y++)
      {
        m_fOctaWeight_C[y*(iwidth>>iScaleX)+x]=-1;
      }
    }

    for (x=0;x<iFaceWidth;x++)
    {
      for (y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[y*iFaceWidth+x]=-1;
      }
    }

    for (x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        fWeightRotZero_C[y*(iFaceWidth>>iScaleX)+x]=-1;
      }
    }

    for (y=0;y<iFaceHeight;y++)
    {
      Int x0, x1;

      getInsideTriangleBoundary(&x0, &x1, y, 0, 0, iFaceWidth, iFaceHeight);

      for (x=0;x<iFaceWidth;x++)
      {
        if (x0 <= x && x <= x1)
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x]=weight;
          weightSum_Y+=weight;
        }
      }
    }

    //Normalize per face_Y
    for (x=0;x<iFaceWidth;x++)
    {
      for (y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[iFaceWidth*y+x]=fWeightRotZero_Y[iFaceWidth*y+x]/weightSum_Y/8.0;
      }
    }
     
    Int iScaledFaceWidth  = iFaceWidth >> iScaleX;
    Int iScaledFaceHeight = iFaceHeight >> iScaleY;
    Int iScaledWidth      = iwidth >> iScaleX;
    Int iScaledHeight     = iheight >> iScaleY;

    for (y=0;y < iScaledFaceHeight;y++)
    {
      Int x0, x1;

      getInsideTriangleBoundary(&x0, &x1, y, iScaleX, iScaleY, iFaceWidth, iFaceHeight);

      for (x=0;x < iScaledFaceWidth;x++)
      {
        if(x0 <= x && x <= x1)
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2)));
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=weight;
          weightSum_C+=weight;
        }
      }
    }

    //Normalize per face_C
    for (x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (y=0;y<(iFaceHeight>>iScaleY);y++)
      {
         fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]/weightSum_C/8.0;
      }
    }

    {

      //weights for the entire frame_Y
      Int start_y;

      for(i = 0, start_y = -(iFaceWidth >> 1);i < 2;i++, start_y += iFaceWidth + 4)
      {
        for(x = 0;x < iFaceWidth;x++)
        {
          Int start_line = start_y + iFaceWidth - 1 - x;
          Int start_offset;
        
          if (start_line < 0)
            start_line += iheight;
          else if (start_line >= iheight)
            start_line -= iheight;

          start_offset = start_line * iwidth;

          for(y = 0;y < iFaceHeight;y++)
          {
            if (fWeightRotZero_Y[iFaceWidth * y + x] >= 0)
            {
              m_fOctaWeight_Y[start_offset + y] = fWeightRotZero_Y[iFaceWidth * y + x];
            }
          }
        }
      }

      for(i = 0, start_y = 2;i < 2;i++, start_y += iFaceWidth + 4)
      {
        for(x = 0;x < iFaceWidth;x++)
        {
          Int start_line = start_y + iFaceWidth - 1 - x;
          Int start_offset;

          start_offset = start_line * iwidth;

          for(y = 0;y < iFaceHeight;y++)
          {
            if (fWeightRotZero_Y[iFaceWidth * y + (iFaceWidth-x-1)] >= 0)
            {
              m_fOctaWeight_Y[start_offset + iFaceHeight - 1 - y] = fWeightRotZero_Y[iFaceWidth * y + (iFaceWidth-x-1)];
            }
          }
        }
      }

      for(y = 0;y < iheight;y++)
        for(x = iwidth >> 1;x < iwidth;x++)
        {
          m_fOctaWeight_Y[iwidth * y + x] = m_fOctaWeight_Y[iwidth * y + (iwidth - 1 - x)];
        }

      //weights for the entire frame_C
      for(i = 0, start_y = -(iScaledFaceWidth >> 1);i < 2;i++, start_y += iScaledFaceWidth + 2)
      {
        for(x = 0;x < iScaledFaceWidth;x++)
        {
          Int start_line = start_y + iScaledFaceWidth - 1 - x;
          Int start_offset;
        
          if (start_line < 0)
            start_line += iScaledHeight;
          else if (start_line >= iScaledHeight)
            start_line -= iScaledHeight;

          start_offset = start_line * iScaledWidth;

          for(y = 0;y < iScaledFaceHeight;y++)
          {
            if (fWeightRotZero_C[iScaledFaceWidth * y + x] >= 0)
              m_fOctaWeight_C[start_offset + y] = fWeightRotZero_C[iScaledFaceWidth * y + x];
          }
        }
      }

      for(i = 0, start_y = 1;i < 2;i++, start_y += iScaledFaceWidth + 2)
      {
        for(x = 0;x < iScaledFaceWidth;x++)
        {
          Int start_line = start_y + iScaledFaceWidth - 1 - x;
          Int start_offset;

          start_offset = start_line * iScaledWidth;

          for(y = 0;y < iScaledFaceHeight;y++)
          {
            if (fWeightRotZero_C[iScaledFaceWidth * y + (iScaledFaceWidth-x-1)] >= 0)
            {
              m_fOctaWeight_C[start_offset + iScaledFaceHeight - 1 - y] = fWeightRotZero_C[iScaledFaceWidth * y + (iScaledFaceWidth-x-1)];
            }
          }
        }
      }

      for(y = 0;y < iScaledHeight;y++)
        for(x = iScaledWidth >> 1;x < iScaledWidth;x++)
        {
          m_fOctaWeight_C[iScaledWidth * y + x] = m_fOctaWeight_C[iScaledWidth * y + (iScaledWidth - 1 - x)];
        }
    }
      
    free(fWeightRotZero_Y);    
    free(fWeightRotZero_C);
#endif
  }
#else
  {
    Int   iwidth  = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iheight = pcPicD->getHeight(COMPONENT_Y) ;
    Double* fWeightRotZero_Y=(Double*)malloc(iFaceWidth*iFaceHeight*sizeof(Double));
    Double* fWeightRotZero_C=(Double*)malloc((iFaceWidth>>iScaleX)*(iFaceHeight>>iScaleY)*sizeof(Double));
    m_fOctaWeight_Y=(Double*)malloc(iwidth*iheight*sizeof(Double));
    m_fOctaWeight_C=(Double*)malloc((iwidth>>iScaleX)*(iheight>>iScaleY)*sizeof(Double));
    Double weightSum_Y=0;
    Double weightSum_C=0;

    for (Int x=0;x<iwidth;x++)
    {
      for (Int y=0;y<iheight;y++)
      {
        m_fOctaWeight_Y[y*iwidth+x]=0;
      }
    }

    for (Int x=0;x<(iwidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iheight>>iScaleY);y++)
      {
        m_fOctaWeight_C[y*(iwidth>>iScaleX)+x]=0;
      }
    }

    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        if(sfabs(x+0.5-iFaceWidth/2) < (y+0.5)/ssqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x]=weight;
          weightSum_Y+=weight;
        }
        else
        {
          fWeightRotZero_Y[iFaceWidth*y+x]=0;
        }
      }
    }

    //Normalize per face_Y
    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[iFaceWidth*y+x]=fWeightRotZero_Y[iFaceWidth*y+x]/weightSum_Y/8.0;
      }
    }
    //weights for the entire frame_Y
    for(Int i=-1;i<2;i++)
      for (Int x=0;x<iFaceWidth;x++)
        for (Int y=0;y<iFaceHeight;y++)
        {
          if(sfabs(x+0.5-iFaceWidth/2) < (y+0.5)/ssqrt(3.0) && 4*i+4+i*iFaceWidth+x+iFaceWidth/2>=0 && 4*i+4+i*iFaceWidth+x+iFaceWidth/2< iwidth)
          {
            m_fOctaWeight_Y[y*iwidth+4*i+4+i*iFaceWidth+x+iFaceWidth/2]=fWeightRotZero_Y[iFaceWidth*y+x];
          }
        }

    for(Int i=0;i<2;i++)
      for(Int x=0;x<iFaceWidth;x++)
        for (Int y=0;y<iFaceHeight;y++)
        {
          if(sfabs(x + 0.5 - iFaceWidth / 2) < (iFaceHeight - (y + 0.5)) / ssqrt(3.0) )
          {
            m_fOctaWeight_Y[y*iwidth+4*i+2+i*iFaceWidth+x]=fWeightRotZero_Y[(iFaceHeight-y-1)*iFaceWidth+(iFaceWidth-x-1)];
          }
        }

    for(Int x=0;x<iwidth;x++)
      for (Int y=0;y<iheight/2;y++)
      {
        m_fOctaWeight_Y[iwidth * iheight / 2 + y * iwidth + x] = m_fOctaWeight_Y[(iheight / 2 - 1 - y) * iwidth + (iwidth - 1 - x)];
      }
      free(fWeightRotZero_Y);

    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (y*(1<<iScaleY)+dChromaOffset[1]+0.5)/ssqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2)));
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=weight;
          weightSum_C+=weight;
        }
        else
        {
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=0;
        }
      }
    }

      //Normalize per face_C
    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]/weightSum_C/8.0;
      }
    }
    //weights for the entire frame_C
    for(Int i=-1;i<2;i++)
      for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
        for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
        {
          if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (y*(1<<iScaleY)+dChromaOffset[1]+0.5)/sqrt(3.0)&& 4*i+4+i*m_iCodingFaceWidth+2*x+m_iCodingFaceWidth/2>=0 && 4*i+4+i*m_iCodingFaceWidth+2*x+m_iCodingFaceWidth/2< iwidth)
          {
            m_fOctaWeight_C[y * (iwidth>>iScaleX) + ((4 * i + 4) >> iScaleX) + i * (iFaceWidth>>iScaleX)+x+(iFaceWidth>>iScaleX)/2] = fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x];
          }
        }


    for(Int i=0;i<2;i++)
      for(Int x=0;x<(iFaceWidth>>iScaleX);x++)
        for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
        {
          if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (iFaceHeight-(y*(1<<iScaleY)+dChromaOffset[1]+0.5))/ssqrt(3.0))
          {
            m_fOctaWeight_C[y * (iwidth >> iScaleX)+((4*i+2)>>iScaleX)+i*(iFaceWidth>>iScaleX)+x]=fWeightRotZero_C[((iFaceHeight>>iScaleY)-y-1)*(iFaceWidth>>iScaleX)+(iFaceWidth>>iScaleX)-x-1];
          }
        }

        for(Int x=0;x<(iwidth>>iScaleX);x++)
          for (Int y=0;y<(iheight>>iScaleY)/2;y++)
          {
            m_fOctaWeight_C[(iwidth>>iScaleX)*(iheight>>iScaleY)/2+y*(iwidth>>iScaleX)+x]=m_fOctaWeight_C[((iheight >> iScaleY)/2-1-y)*(iwidth>>iScaleX)+((iwidth>>iScaleX)-1-x)];
          }

          free(fWeightRotZero_C);
  }
#endif
  else if(pcCodingGeomtry->getType()==SVIDEO_OCTAHEDRON && pCodingSVideoInfo->iCompactFPStructure ==2)
  {
    Int   iwidth  = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iheight = pcPicD->getHeight(COMPONENT_Y) ;
    Double* fWeightRotZero_Y=(Double*)malloc(iFaceWidth*iFaceHeight*sizeof(Double));
    Double* fWeightRotZero_C=(Double*)malloc((iFaceWidth>>iScaleX)*(iFaceHeight>>iScaleY)*sizeof(Double));
    m_fOctaWeight_Y=(Double*)malloc(iwidth*iheight*sizeof(Double));
    m_fOctaWeight_C=(Double*)malloc((iwidth>>iScaleX)*(iheight>>iScaleY)*sizeof(Double));
    Double weightSum_Y=0;
    Double weightSum_C=0;

    for (Int x=0;x<iwidth;x++)
    {
      for (Int y=0;y<iheight;y++)
      {
        m_fOctaWeight_Y[y*iwidth+x]=0;
      }
    }

    for (Int x=0;x<(iwidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iheight>>iScaleY);y++)
      {
        m_fOctaWeight_C[y*(iwidth>>iScaleX)+x]=0;
      }
    }

    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        if(sfabs(x+0.5-iFaceWidth/2) < (y+0.5)/ssqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x]=weight;
          weightSum_Y+=weight;
        }
        else
        {
          fWeightRotZero_Y[iFaceWidth*y+x]=0;
        }
      }
    }

    //Normalize per face_Y
    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[iFaceWidth*y+x]=fWeightRotZero_Y[iFaceWidth*y+x]/weightSum_Y/8.0;
      }
    }
    //weights for the entire frame_Y
    for(Int i=0;i<4;i++)
     for (Int x=0;x<iFaceWidth;x++)
      for (Int y=0;y<iFaceHeight;y++)
      {
        if(sfabs(x+0.5-iFaceWidth/2) < (y+0.5)/ssqrt(3.0))
        {
          m_fOctaWeight_Y[y*iwidth+4*i+2+i*iFaceWidth+x]=fWeightRotZero_Y[iFaceWidth*y+x];
        }
      }

    for(Int i=-1;i<4;i++)
     for(Int x=0;x<iFaceWidth;x++)
      for (Int y=0;y<iFaceHeight;y++)
      {
        if(sfabs(x+0.5-iFaceWidth/2) < (iFaceHeight-(y+0.5))/ssqrt(3.0) && 4*i+4+i*iFaceWidth+x+iFaceWidth/2>=0 && 4*i+4+i*iFaceWidth+x+iFaceWidth/2< iwidth)
        {
          m_fOctaWeight_Y[y*iwidth+4*i+4+i*iFaceWidth+x+iFaceWidth/2]=fWeightRotZero_Y[(iFaceHeight-y-1)*iFaceWidth+x];
        }
      }

    free(fWeightRotZero_Y);

    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (y*(1<<iScaleY)+dChromaOffset[1]+0.5)/ssqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
          r2 =(iFaceWidth/ssqrt(6.0))*(iFaceWidth/ssqrt(6.0));
          Double weight = 1.0/((1+d2/r2)*ssqrt(1.0*(1+d2/r2)));
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=weight;
          weightSum_C+=weight;
        }
        else
        {
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=0;
        }
      }
    }

    //Normalize per face_C
    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]/weightSum_C/8.0;
      }
    }
    //weights for the entire frame_C
    for(Int i=0;i<4;i++)
      for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
        for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
        {
          if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (y*(1<<iScaleY)+dChromaOffset[1]+0.5)/sqrt(3.0))
          {
            m_fOctaWeight_C[y*(iwidth >> iScaleX)+2*i+1+i*(iFaceWidth>>iScaleX)+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x];
          }
        }

        for(Int i=-1;i<4;i++)
          //for(Int x=0;x<m_iCodingFaceWidth;x++)             //WS-PSNR checking;
          for(Int x=0;x<(iFaceWidth>>iScaleX);x++)
            for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
            {
              if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (iFaceHeight-(y*(1<<iScaleY)+dChromaOffset[1]+0.5))/ssqrt(3.0) && 4*i+4+i*iFaceWidth+(x<<iScaleX)+iFaceWidth/2>=0 && 4*i+4+i*iFaceWidth+(x<<iScaleX)+iFaceWidth/2< iwidth)
              {
                m_fOctaWeight_C[y*(iwidth>>iScaleX)+2*i+2+i*(iFaceWidth>>iScaleX)+x+((iFaceWidth/2)>>iScaleX)]=fWeightRotZero_C[((iFaceHeight>>iScaleY)-y-1)*(iFaceWidth>>iScaleX)+x];
              }
            }
    free(fWeightRotZero_C);
  }
  else if(pcCodingGeomtry->getType()==SVIDEO_ICOSAHEDRON && pCodingSVideoInfo->iCompactFPStructure ==0)
  {
    Int iwidth=iFaceWidth*pCodingSVideoInfo->framePackStruct.cols;
    Int iheight=iFaceHeight*pCodingSVideoInfo->framePackStruct.rows;
    Double* fWeightRotZero_Y=(Double*)malloc(iFaceWidth*iFaceHeight*sizeof(Double));
    Double* fWeightRotZero_C=(Double*)malloc((iFaceWidth>>iScaleX)*(iFaceHeight>>iScaleY)*sizeof(Double));
    m_fIcoWeight_Y=(Double*)malloc(iwidth*iheight*sizeof(Double));
    m_fIcoWeight_C=(Double*)malloc((iwidth>>iScaleX)*(iheight>>iScaleY)*sizeof(Double));
    Double Ratio_r= 4*ssqrt(3.0)/(3+ssqrt(5.0));
    Double weightSum_Y=0;
    Double weightSum_C=0;

    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        if(sfabs(x+0.5-iFaceWidth/2) < (y+0.5)/ssqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
          r2 =(iFaceWidth/Ratio_r)*(iFaceWidth/Ratio_r);
          Double weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x]=weight;
          weightSum_Y+=weight;
        }
        else
        {
          fWeightRotZero_Y[iFaceWidth*y+x]=0;
        }
      }
    }

    //Normalize per face_Y
    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[iFaceWidth*y+x]=fWeightRotZero_Y[iFaceWidth*y+x]/weightSum_Y/20.0;
      }
    }

    //weights for the entire frame_Y
    Double weight;
    for (Int rows=0;rows<pCodingSVideoInfo->framePackStruct.rows;rows++)
    {
      Int ypos=rows*iFaceHeight;
      for (Int cols=0;cols<pCodingSVideoInfo->framePackStruct.cols;cols++)
      {
        Int xpos=cols*iFaceWidth;
        for (Int x=0;x<iFaceWidth;x++)
        {
          for (Int y=0;y<iFaceHeight;y++)
          {
            if(pCodingSVideoInfo->framePackStruct.faces[rows][cols].rot==0)
            {
              weight=fWeightRotZero_Y[y*iFaceWidth+x];
            }
            else
            {
              weight=fWeightRotZero_Y[(iFaceHeight-y-1)*iFaceWidth+(iFaceWidth-x-1)];
            }
            m_fIcoWeight_Y[(ypos+y)*iwidth+xpos+x]=weight;
          }
        }
      }
    }
    free(fWeightRotZero_Y);

    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        if(sfabs(x*(1<<iScaleX)+dChromaOffset[0]+0.5-iFaceWidth/2) < (y*(1<<iScaleY)+dChromaOffset[1]+0.5)/sqrt(3.0))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
          r2 =(iFaceWidth/Ratio_r)*(iFaceWidth/Ratio_r);
          weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=weight;
          weightSum_C+=weight;
        }
        else
        {
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=0;
        }
      }
    }

    //Normalize per face_C
    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]/weightSum_C/20.0;
      }
    }
    //weights for the entire frame_C
    for (Int rows=0;rows<pCodingSVideoInfo->framePackStruct.rows;rows++)
    {
      Int ypos=rows*(iFaceHeight>>iScaleY);
      for (Int cols=0;cols<pCodingSVideoInfo->framePackStruct.cols;cols++)
      {
        Int xpos=cols*(iFaceWidth>>iScaleX);
        for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
        {
          for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
          {
            if(pCodingSVideoInfo->framePackStruct.faces[rows][cols].rot==0)
            {
              weight=fWeightRotZero_C[y*(iFaceWidth>>iScaleX)+x];
            }
            else
            {
              weight=fWeightRotZero_C[((iFaceHeight>>iScaleY)-y-1)*(iFaceWidth>>iScaleX)+((iFaceWidth>>iScaleX)-1-x)];
            }
            m_fIcoWeight_C[(ypos+y)*(iwidth>>iScaleX)+xpos+x]=weight;
          }
        }
      }
    }
    free(fWeightRotZero_C);
  }
#if  SVIDEO_WSPSNR_ISP1
  else if(pcCodingGeomtry->getType()==SVIDEO_ICOSAHEDRON && pCodingSVideoInfo->iCompactFPStructure ==1)
  {
    Int   iwidth  = pcPicD->getWidth(COMPONENT_Y) ;
    Int   iheight = pcPicD->getHeight(COMPONENT_Y) ;
    Double* fWeightRotZero_Y=(Double*)malloc(iFaceWidth*iFaceHeight*sizeof(Double));
    Double* fWeightRotZero_C=(Double*)malloc((iFaceWidth>>iScaleX)*(iFaceHeight>>iScaleY)*sizeof(Double));
    m_fIcoWeight_Y=(Double*)malloc(iwidth*iheight*sizeof(Double));
    m_fIcoWeight_C=(Double*)malloc((iwidth>>iScaleX)*(iheight>>iScaleY)*sizeof(Double));
    Double Ratio_r= 4*ssqrt(3.0)/(3+ssqrt(5.0));
    Double weightSum_Y=0;
    Double weightSum_C=0;

    for (Int x=0;x<iwidth;x++)
    {
      for (Int y=0;y<iheight;y++)
      {
        m_fIcoWeight_Y[y*iwidth+x]=0;
      }
    }

    for (Int x=0;x<(iwidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iheight>>iScaleY);y++)
      {
        m_fIcoWeight_C[y*(iwidth>>iScaleX)+x]=0;
      }
    }

    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        if(pcCodingGeomtry->insideFace(0, x, y, COMPONENT_Y, COMPONENT_Y))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x+0.5-ci)*(x+0.5-ci)+(y+0.5-cj)*(y+0.5-cj);
          r2 =(iFaceWidth/Ratio_r)*(iFaceWidth/Ratio_r);
          Double weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_Y[iFaceWidth*y+x]=weight;
          weightSum_Y+=weight;
        }
        else
        {
          fWeightRotZero_Y[iFaceWidth*y+x]=0;
        }
      }
    }

    //Normalize per face_Y
    for (Int x=0;x<iFaceWidth;x++)
    {
      for (Int y=0;y<iFaceHeight;y++)
      {
        fWeightRotZero_Y[iFaceWidth*y+x]=fWeightRotZero_Y[iFaceWidth*y+x]/weightSum_Y/20.0;
      }
    }

    //weights for the entire frame_Y
    for(Int faceIdx=0; faceIdx<20; faceIdx++)
    {
      for(Int v=0; v<iFaceHeight; v++)
        for(Int u=0; u<iFaceWidth; u++)
          if(pcCodingGeomtry->insideFace(faceIdx, u, v, COMPONENT_Y, COMPONENT_Y))
          {
            IPos tmpPos;
            tmpPos.faceIdx = faceIdx;
            tmpPos.u = u;
            tmpPos.v = v;
            IPos2D framePackedPos;
            pcCodingGeomtry->geoToFramePack(&tmpPos, &framePackedPos);    
            m_fIcoWeight_Y[framePackedPos.y*iwidth + framePackedPos.x] = fWeightRotZero_Y[v*iFaceWidth+u];
          }
    }
    free(fWeightRotZero_Y);

    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        if(pcCodingGeomtry->insideFace(0, (x<<iScaleX), (y<<iScaleY), COMPONENT_Y, COMPONENT_Cb))
        {
          Int ci=0;
          Int cj=0;
          Double r2,d2;
          ci = iFaceWidth/2;
          cj = 2*iFaceHeight/3;
          d2 =(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci)*(x*(1<<iScaleX)+dChromaOffset[0]+0.5-ci) + (y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj)*(y*(1<<iScaleY)+dChromaOffset[1]+0.5-cj);
          r2 =(iFaceWidth/Ratio_r)*(iFaceWidth/Ratio_r);
          Double weight = 1.0/((1+d2/r2)*sqrt(1.0*(1+d2/r2)));
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=weight;
          weightSum_C+=weight;
        }
        else
        {
          fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=0;
        }
      }
    }

    //Normalize per face_C
    for (Int x=0;x<(iFaceWidth>>iScaleX);x++)
    {
      for (Int y=0;y<(iFaceHeight>>iScaleY);y++)
      {
        fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]=fWeightRotZero_C[(iFaceWidth>>iScaleX)*y+x]/weightSum_C/20.0;
      }
    }
    //weights for the entire frame_C
    for(Int faceIdx=0; faceIdx<20; faceIdx++)
    {
      for(Int v=0; v<(iFaceHeight>>iScaleY); v++)
        for(Int u=0; u<(iFaceWidth>>iScaleX); u++)
          if(pcCodingGeomtry->insideFace(faceIdx, (u<<iScaleX), (v<<iScaleY), COMPONENT_Y, COMPONENT_Cb))
          {
            IPos tmpPos;
            tmpPos.faceIdx = faceIdx;
            tmpPos.u = u<<iScaleX;
            tmpPos.v = v<<iScaleY;
            IPos2D framePackedPos;
            pcCodingGeomtry->geoToFramePack(&tmpPos, &framePackedPos);     
            m_fIcoWeight_C[(framePackedPos.y>>iScaleY)*(iwidth>>iScaleY) + (framePackedPos.x>>iScaleX)] = fWeightRotZero_C[v*(iFaceWidth>>iScaleX)+u];
          }
    }
    free(fWeightRotZero_C);

  }
#endif
#if SVIDEO_WSPSNR_SSP
#if SVIDEO_FIX_TICKET47
  else if (pcCodingGeomtry->getType() == SVIDEO_SEGMENTEDSPHERE)
  {
    Int iwidth = iFaceWidth*pCodingSVideoInfo->framePackStruct.cols;
    Int iheight = iFaceHeight*pCodingSVideoInfo->framePackStruct.rows;
#if SVIDEO_EAP_SSP_PADDING
    iheight += (SVIDEO_SSP_GUARD_BAND << 2);
#endif
    m_fSspWeight_Y = (Double*)malloc(iwidth*iheight * sizeof(Double));
    m_fSspWeight_C = (Double*)malloc((iwidth >> iScaleX)*(iheight >> iScaleY) * sizeof(Double));
    Double weightSum_Y = 0;
    Double weightSum_C = 0;

    for (Int u = 0; u<iwidth; u++)
    {
      for (Int v = 0; v<iheight; v++)
      {
        m_fSspWeight_Y[iwidth*v + u] = 0;
      }
    }

    for (Int faceIdx = 0; faceIdx<6; faceIdx++)
    {
      for (Int v = 0; v<iFaceHeight; v++)
        for (Int u = 0; u<iFaceWidth; u++)
          if (pcCodingGeomtry->insideFace(faceIdx, u, v, COMPONENT_Y, COMPONENT_Y))
          {
            SPos tmpPos;
            tmpPos.faceIdx = faceIdx;
            tmpPos.x = u;
            tmpPos.y = v;
            tmpPos.z = 0;
            SPos IPosOut;
            pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);

            Double pitch;
            pitch = sasin(IPosOut.y);

            IPos inPos;
            inPos.faceIdx = faceIdx;
            inPos.u = u;
            inPos.v = v;
            IPos2D framePos;
            pcCodingGeomtry->geoToFramePack(&inPos, &framePos);
            if (faceIdx == 0 || faceIdx == 1)
            {
              m_fSspWeight_Y[framePos.x + framePos.y*iwidth] = scos(fabs(pitch)) / (S_PI_2 - fabs(pitch));
            }
            else
            {
#if SVIDEO_EAP_SSP_PADDING
              m_fSspWeight_Y[framePos.x + framePos.y*iwidth] = 2.0 * ssqrt(2.0) / S_PI;
#else
              m_fSspWeight_Y[framePos.x + framePos.y*iwidth] = scos(pitch);
#endif
            }
            weightSum_Y += m_fSspWeight_Y[framePos.x + framePos.y*iwidth];
          }
    }

    for (Int x = 0; x<iwidth; x++)
    {
      for (Int y = 0; y<iheight; y++)
      {
        m_fSspWeight_Y[iwidth*y + x] = m_fSspWeight_Y[iwidth*y + x] / weightSum_Y;
      }
    }



    for (Int u = 0; u<iwidth >> iScaleX; u++)
    {
      for (Int v = 0; v<iheight >> iScaleY; v++)
      {
        m_fSspWeight_C[(iwidth >> iScaleX)*v + u] = 0;
      }
    }

    for (Int faceIdx = 0; faceIdx<6; faceIdx++)
    {
      for (Int v = 0; v<iFaceHeight >> iScaleY; v++)
        for (Int u = 0; u<iFaceWidth >> iScaleX; u++)
          if (pcCodingGeomtry->insideFace(faceIdx, u << iScaleX, v << iScaleY, COMPONENT_Y, COMPONENT_Cb))
          {
            SPos tmpPos;
            tmpPos.faceIdx = faceIdx;
            tmpPos.x = (u << iScaleX) + dChromaOffset[0];
            tmpPos.y = (v << iScaleY) + dChromaOffset[1];
            tmpPos.z = 0;
            SPos IPosOut;
            pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);

            Double pitch;
            pitch = sasin(IPosOut.y);

            IPos inPos;
            inPos.faceIdx = faceIdx;
            inPos.u = u << iScaleX;
            inPos.v = v << iScaleY;
            IPos2D framePos;
            pcCodingGeomtry->geoToFramePack(&inPos, &framePos);
            if (faceIdx == 0 || faceIdx == 1)
            {
              m_fSspWeight_C[(framePos.x >> iScaleX) + (framePos.y >> iScaleY)*(iwidth >> iScaleX)] = scos(fabs(pitch)) / (S_PI_2 - fabs(pitch));
            }
            else
            {
#if SVIDEO_EAP_SSP_PADDING
              m_fSspWeight_C[(framePos.x >> iScaleX) + (framePos.y >> iScaleY)*(iwidth >> iScaleX)] = 2.0 * ssqrt(2.0) / S_PI;
#else
              m_fSspWeight_C[(framePos.x >> iScaleX) + (framePos.y >> iScaleY)*(iwidth >> iScaleX)] = scos(pitch);
#endif
            }
            weightSum_C += m_fSspWeight_C[(framePos.x >> iScaleX) + (framePos.y >> iScaleY)*(iwidth >> iScaleX)];
          }
    }

    for (Int x = 0; x<iwidth >> iScaleX; x++)
    {
      for (Int y = 0; y<iheight >> iScaleY; y++)
      {
        m_fSspWeight_C[(iwidth >> iScaleX)*y + x] = m_fSspWeight_C[(iwidth >> iScaleX)*y + x] / weightSum_C;
      }
    }
  }
#else
  else if (pcCodingGeomtry->getType() == SVIDEO_SEGMENTEDSPHERE)
  {
      Int iwidth = iFaceWidth*pCodingSVideoInfo->framePackStruct.cols;
      Int iheight = iFaceHeight*pCodingSVideoInfo->framePackStruct.rows;
      m_fSspWeight_Y = (Double*)malloc(iwidth*iheight * sizeof(Double));
      m_fSspWeight_C = (Double*)malloc((iwidth >> iScaleX)*(iheight >> iScaleY) * sizeof(Double));
      Double weightSum_Y = 0;
      Double weightSum_C = 0;

      for (Int u = 0; u<iwidth; u++)
      {
          for (Int v = 0; v<iheight; v++)
          {
              m_fSspWeight_Y[iwidth*v + u] = 0;
          }
      }

      for (Int faceIdx = 0; faceIdx<6; faceIdx++)
      {
          for (Int v = 0; v<iFaceHeight; v++)
              for (Int u = 0; u<iFaceWidth; u++)
                  if (pcCodingGeomtry->insideFace(faceIdx, u, v, COMPONENT_Y, COMPONENT_Y))
                  {
                      SPos tmpPos;
                      tmpPos.faceIdx = faceIdx;
#if SVIDEO_SSP_VERT
                      Int* pos = pcCodingGeomtry->getFacePos(faceIdx);
                      if ((pCodingSVideoInfo->framePackStruct.faces[pos[0]][pos[1]].rot % 180) == 90)
                      {
                          tmpPos.x = iFaceHeight - 1 - v;
                          tmpPos.y = u;
                          tmpPos.z = 0;
                          SPos IPosOut;
                          Double pitch;
                          pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                          pitch = sasin(IPosOut.y);
                          if (faceIdx == 0)
                          {
                              m_fSspWeight_Y[(v + iFaceHeight*faceIdx)*iwidth + u] = scos(pitch) / (S_PI_2 - pitch);
                          }
                          else if (faceIdx == 1)
                          {
                              m_fSspWeight_Y[(v + iFaceHeight*faceIdx)*iwidth + u] = m_fSspWeight_Y[v*iwidth + u];
                          }
                          else
                          {
                              m_fSspWeight_Y[(v + iFaceHeight*faceIdx)*iwidth + u] = scos(pitch);
                          }
                          weightSum_Y += m_fSspWeight_Y[(v + iFaceHeight*faceIdx)*iwidth + u];
                      }
                      else
                      {
                          tmpPos.x = u;
                          tmpPos.y = v;
                          tmpPos.z = 0;
                          SPos IPosOut;
                          Double pitch;
                          pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                          pitch = sasin(IPosOut.y);
                          if (faceIdx == 0)
                          {
                              m_fSspWeight_Y[v*iwidth + u] = cos(pitch) / (S_PI_2 - pitch);
                          }
                          else if (faceIdx == 1)
                          {
                              m_fSspWeight_Y[v*iwidth + iFaceWidth*faceIdx + u] = m_fSspWeight_Y[v*iwidth + u];
                          }
                          else
                          {
                              m_fSspWeight_Y[v*iwidth + iFaceWidth*faceIdx + u] = scos(pitch);
                          }
                          weightSum_Y += m_fSspWeight_Y[v*iwidth + iFaceWidth*faceIdx + u];
                      }
#else
                      tmpPos.x = u;
                      tmpPos.y = v;
                      tmpPos.z = 0;
                      SPos IPosOut;
                      Double pitch;
                      pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                      pitch = sasin(IPosOut.y);
                      if (faceIdx == 0)
                      {
                          m_fSspWeight_Y[v*iwidth + u] = 16 * cos(pitch) / (S_PI*S_PI*(iheight / 2)*(iheight / 2)) / (S_PI_2 - pitch);
                      }
                      else if (faceIdx == 1)
                      {
                          m_fSspWeight_Y[v*iwidth + iFaceWidth*faceIdx + u] = m_fSspWeight_Y[v*iwidth + u];
                      }
                      else
                      {
                          m_fSspWeight_Y[v*iwidth + iFaceWidth*faceIdx + u] = (S_PI*S_PI)*cos(pitch) / (iheight*iheight * 4);
                      }
                      weightSum_Y += m_fSspWeight_Y[v*iwidth + iFaceWidth*faceIdx + u];
#endif
                  }
      }

      for (Int x = 0; x<iwidth; x++)
      {
          for (Int y = 0; y<iheight; y++)
          {
              m_fSspWeight_Y[iwidth*y + x] = m_fSspWeight_Y[iwidth*y + x] / weightSum_Y;
          }
      }


      for (Int u = 0; u<iwidth >> iScaleX; u++)
      {
          for (Int v = 0; v<iheight >> iScaleY; v++)
          {
              m_fSspWeight_C[(iwidth >> iScaleX)*v + u] = 0;
          }
      }

      for (Int faceIdx = 0; faceIdx<6; faceIdx++)
      {
          for (Int v = 0; v<iFaceHeight >> iScaleY; v++)
              for (Int u = 0; u<iFaceWidth >> iScaleX; u++)
                  if (pcCodingGeomtry->insideFace(faceIdx, u << iScaleX, v << iScaleY, COMPONENT_Y, COMPONENT_Cb))
                  {
#if SVIDEO_SSP_VERT
                      Int* pos = pcCodingGeomtry->getFacePos(faceIdx);
                      if ((pCodingSVideoInfo->framePackStruct.faces[pos[0]][pos[1]].rot % 180) == 90)
                      {
                          SPos tmpPos;
                          tmpPos.faceIdx = faceIdx;
                          tmpPos.x = iFaceHeight - 1 - ((v << iScaleY) + dChromaOffset[1]);
                          tmpPos.y = (u << iScaleX) + dChromaOffset[0];
                          tmpPos.z = 0;
                          SPos IPosOut;
                          Double pitch;
                          pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                          pitch = sasin(IPosOut.y);

                          if (faceIdx == 0)
                          {
                              m_fSspWeight_C[v*(iwidth >> iScaleX) + u] = scos(pitch) / (S_PI_2 - pitch);
                          }
                          else if (faceIdx == 1)
                          {
                              m_fSspWeight_C[(v + (iFaceHeight >> iScaleY)*faceIdx)*(iwidth >> iScaleX) + u] = m_fSspWeight_C[v*(iwidth >> iScaleX) + u];
                          }
                          else
                          {
                              m_fSspWeight_C[(v + (iFaceHeight >> iScaleY)*faceIdx)*(iwidth >> iScaleX) + u] = scos(pitch);
                          }
                          weightSum_C += m_fSspWeight_C[(v + (iFaceHeight >> iScaleY)*faceIdx)*(iwidth >> iScaleX) + u];
                      }
                      else
                      {
                          SPos tmpPos;
                          tmpPos.faceIdx = faceIdx;
                          tmpPos.x = (u << iScaleX) + dChromaOffset[0];
                          tmpPos.y = (v << iScaleY) + dChromaOffset[1];
                          tmpPos.z = 0;
                          SPos IPosOut;
                          Double pitch;
                          pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                          pitch = sasin(IPosOut.y);

                          if (faceIdx == 0)
                          {
                              m_fSspWeight_C[v*(iwidth >> iScaleX) + u] = cos(pitch) / (S_PI_2 - pitch);
                          }
                          else if (faceIdx == 1)
                          {
                              m_fSspWeight_C[v*(iwidth >> iScaleX) + (iFaceWidth >> iScaleX)*faceIdx + u] = m_fSspWeight_C[v*(iwidth >> iScaleX) + u];
                          }
                          else
                          {
                              m_fSspWeight_C[v*(iwidth >> iScaleX) + (iFaceWidth >> iScaleX)*faceIdx + u] = cos(pitch);
                          }
                          weightSum_C += m_fSspWeight_C[v*(iwidth >> iScaleX) + (iFaceWidth >> iScaleX)*faceIdx + u];
                      }
#else
                      SPos tmpPos;
                      tmpPos.faceIdx = faceIdx;
                      tmpPos.x = (u << iScaleX) + dChromaOffset[0];
                      tmpPos.y = (v << iScaleY) + dChromaOffset[1];
                      tmpPos.z = 0;
                      SPos IPosOut;
                      Double pitch;
                      pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                      pitch = sasin(IPosOut.y);

                      if (faceIdx == 0)
                      {
                          m_fSspWeight_C[v*(iwidth >> iScaleX) + u] = 16 * cos(pitch) / (S_PI*S_PI*(iheight / 2)*(iheight / 2)) / (S_PI_2 - pitch);
                      }
                      else if (faceIdx == 1)
                      {
                          m_fSspWeight_C[v*(iwidth >> iScaleX) + (iFaceWidth >> iScaleX)*faceIdx + u] = m_fSspWeight_C[v*(iwidth >> iScaleX) + u];
                      }
                      else
                      {
                          m_fSspWeight_C[v*(iwidth >> iScaleX) + (iFaceWidth >> iScaleX)*faceIdx + u] = (S_PI*S_PI)*cos(pitch) / (iheight*iheight * 4);
                      }
                      weightSum_C += m_fSspWeight_C[v*(iwidth >> iScaleX) + (iFaceWidth >> iScaleX)*faceIdx + u];
#endif
                  }
      }

      for (Int x = 0; x<iwidth >> iScaleX; x++)
      {
          for (Int y = 0; y<iheight >> iScaleY; y++)
          {
              m_fSspWeight_C[(iwidth >> iScaleX)*y + x] = m_fSspWeight_C[(iwidth >> iScaleX)*y + x] / weightSum_C;
          }
      }

  }
#endif
#endif
#if SVIDEO_ROTATED_SPHERE
  else if(pcCodingGeomtry->getType()==SVIDEO_ROTATEDSPHERE)
  {
      Int   iWidth = pcPicD->getWidth(COMPONENT_Y) ;
      Int   iHeight = pcPicD->getHeight(COMPONENT_Y);
      
      assert( iWidth  == iFaceWidth*pCodingSVideoInfo->framePackStruct.cols);
      assert( iHeight == iFaceHeight*pCodingSVideoInfo->framePackStruct.rows);
      
      Int   iWidthC = pcPicD->getWidth(COMPONENT_Cb) ;
      Int   iHeightC = pcPicD->getHeight(COMPONENT_Cb);

      Double weightSum_Y = 0;
      Double weightSum_C = 0;
      
      enum
      {
          RSP_FRONT_FACE    = 0,
          RSP_BACK_FACE     = 1,
          RSP_TOP_FACE      = 2,
          RSP_BOTTOM_FACE   = 3,
          RSP_RIGHT_FACE    = 4,
          RSP_LEFT_FACE     = 5,
          RSP_FACE_COUNT    = 6,
      };
      
      m_fRspWeight_Y = (Double*)malloc(iWidth*iHeight * sizeof(Double));
      memset( m_fRspWeight_Y, 0, iWidth*iHeight * sizeof(Double));

      m_fRspWeight_C = (Double*)malloc(iWidthC*iHeightC * sizeof(Double));
      memset( m_fRspWeight_C, 0, iWidthC*iHeightC * sizeof(Double));
      
      Int top_row_faces[] = { RSP_FRONT_FACE, RSP_LEFT_FACE, RSP_RIGHT_FACE };
      
      for (Int i = 0; i < sizeof(top_row_faces) / sizeof(top_row_faces[0]); i++)
      {
          Int faceIdx = top_row_faces[i];
          
          for (Int v = 0; v < iFaceHeight; v++)
              for (Int u = 0; u < iFaceWidth; u++)
              {
#if SVIDEO_RSP_3D_ARC
                  Bool insideFace = pcCodingGeomtry->insideFace(faceIdx, u, v, COMPONENT_Y, COMPONENT_Y);
#else
                  Bool insideFace = true;
                  
                  Int radius = (iFaceHeight >> 1);
                  
                  if( ( u < radius && faceIdx == RSP_RIGHT_FACE ) || ( u > radius &&  faceIdx == RSP_LEFT_FACE ) )
                  {
                      Double x_L = radius - u - 0.5;
                      Double y_L = radius - v - 0.5;
                      Double d = ssqrt(x_L*x_L + y_L*y_L);
                      
                      insideFace = d <= radius;
                  }
#endif
                  if ( insideFace )
                  {
                      Int offset = 0;
                      
                      switch( faceIdx )
                      {
                          case RSP_FRONT_FACE:
                              offset = 1 * iFaceWidth;
                              break;
                              
                          case RSP_LEFT_FACE:
                              offset = 2 * iFaceWidth;
                              break;
                              
                          case RSP_RIGHT_FACE:
                              offset = 0 * iFaceWidth;
                              break;
                              
                          default:
                              assert(0);
                      }
                      
                      Double dWeight;
                      
                      {
                          SPos tmpPos;
                          tmpPos.faceIdx = faceIdx;
                          tmpPos.x = u;
                          tmpPos.y = v;
                          tmpPos.z = 0;
                          SPos IPosOut;
                          Double pitch;
                          pcCodingGeomtry->map2DTo3D(tmpPos, &IPosOut);
                          
                          pitch = sasin(IPosOut.y);
                          
                          dWeight = scos(pitch);
                      }
                      
                      m_fRspWeight_Y[ v*iWidth + u + offset] = dWeight;
                      
                      weightSum_Y += dWeight;
                      
                      m_fRspWeight_C[ (v>>iScaleY)*(iWidthC) + (u>>iScaleX) + (offset>>iScaleX)] = dWeight;
                      
                      weightSum_C += dWeight;
                  }
              }
      }
      
      weightSum_Y *= 2;
      weightSum_C *= 2;
      
      // Luma
      for (Int x = 0; x < iWidth; x++)
      {
          for (Int y = iHeight / 2; y < iHeight; y++)
          {
              m_fRspWeight_Y[iWidth*(y - iHeight / 2) + x] /= weightSum_Y;
              m_fRspWeight_Y[iWidth*y + x] = m_fRspWeight_Y[iWidth*(y - iHeight / 2) + x];
          }
      }
      
      // Chroma
      for (Int x = 0; x < iWidthC; x++)
      {
          for (Int y = iHeightC / 2; y < iHeightC; y++)
          {
              m_fRspWeight_C[iWidthC*(y - iHeightC / 2) + x] /= weightSum_C;
              m_fRspWeight_C[iWidthC*y + x] = m_fRspWeight_C[iWidthC*(y - iHeightC / 2) + x];
          }
      }
  }
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
  else if (pcCodingGeomtry->getType() == SVIDEO_EQUATORIALCYLINDRICAL)
  {
    Double weight = 1.0;
    Double padmargin = 4.0;
    Double *EcpFaceWeight_Y;

    EcpFaceWeight_Y = (Double*)malloc(iFaceHeight * iFaceWidth * sizeof(Double));
    memset(EcpFaceWeight_Y, 0, iFaceHeight * iFaceWidth * sizeof(Double));

    Int   iWidth = pcPicD->getWidth(COMPONENT_Y);
    Int   iHeight = pcPicD->getHeight(COMPONENT_Y);
    Int   iWidthC = pcPicD->getWidth(COMPONENT_Cb) ;
    Int   iHeightC = pcPicD->getHeight(COMPONENT_Cb);
    assert( iWidth  == iFaceWidth*pCodingSVideoInfo->framePackStruct.cols);
    assert( iHeight == iFaceHeight*pCodingSVideoInfo->framePackStruct.rows);

    m_fEcpWeight_Y = (Double*)malloc(iWidth*iHeight * sizeof(Double));
    memset( m_fEcpWeight_Y, 0, iWidth*iHeight * sizeof(Double));
    m_fEcpWeight_C = (Double*)malloc(iWidthC*iHeightC * sizeof(Double));
    memset( m_fEcpWeight_C, 0, iWidthC*iHeightC * sizeof(Double));

    // Faces 0, 1:
    for (Int y = 0; y < (iFaceHeight - (Int)padmargin); y++)
    {
      for (Int x = (Int)padmargin; x < (iFaceWidth - (Int)padmargin); x++)
      {
        SPos SPosIn(0, x, y, 0), SPosOut;
        pcCodingGeomtry->map2DTo3D(SPosIn, &SPosOut);
        weight = CalculateEcpWeight(SPosOut, iFaceWidth);
        EcpFaceWeight_Y[iFaceWidth*y + x] = weight;
      }
    }
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x < iFaceWidth; x++)
      {
        Double w = (EcpFaceWeight_Y[iFaceWidth*y + x]);
        // Face 0
        m_fEcpWeight_Y[(x + iFaceHeight)*iWidth + (iFaceWidth - y - 1)] = w;
        // Face 1
        m_fEcpWeight_Y[(x + iFaceHeight)*iWidth + (iWidth - iFaceWidth + y)] = w;
      }
    }

    Double padfactor = 1.0 + 2.0*padmargin/((Double)iFaceWidth - 2.0*padmargin);
    Double apadfactor = (Double)iFaceWidth/((Double)iFaceWidth - padmargin);
    // Face 2
    for (Int y = (Int)padmargin; y < (iFaceHeight - (Int)padmargin); y++)
    {
      for (Int x = (Int)padmargin; x < iFaceWidth; x++)
      {
        m_fEcpWeight_Y[y*iWidth + x] = S_PI*padfactor*apadfactor/6.0;
      }
    }
    // Face 3
    for (Int y = (Int)padmargin; y < (iFaceHeight - (Int)padmargin); y++)
    {
      for (Int x = 0; x < iFaceWidth; x++)
      {
        m_fEcpWeight_Y[y*iWidth + (iFaceWidth + x)] = S_PI*padfactor/6.0;
      }
    }
    // Face 4
    for (Int y = (Int)padmargin; y < (iFaceHeight - (Int)padmargin); y++)
    {
      for (Int x = 0; x < (iFaceWidth - (Int)padmargin); x++)
      {
        m_fEcpWeight_Y[y*iWidth + (2*iFaceWidth + x)] = S_PI*padfactor*apadfactor/6.0;
      }
    }
    // Face 5
    for (Int y = (Int)padmargin; y < (iFaceHeight - (Int)padmargin); y++)
    {
      for (Int x = 0; x < iFaceWidth; x++)
      {
        m_fEcpWeight_Y[(y + iFaceHeight)*iWidth + (iFaceWidth + x)] = S_PI*padfactor/6.0;
      }
    }

    for (Int y = 0; y < iHeightC; y++)
    {
      for (Int x = 0; x < iWidthC; x++)
      {
        m_fEcpWeight_C[y*iWidthC + x] = m_fEcpWeight_Y[(y<<iScaleY)*iWidth + (x<<iScaleX)];
      }
    }
    free(EcpFaceWeight_Y);
  }
#else
  else if (pcCodingGeomtry->getType() == SVIDEO_EQUATORIALCYLINDRICAL)
  {
    Double fWeightSum_Y = 0;
    Double fWeightSum_C = 0;

    m_fCubeWeight_Y = (Double*)malloc(iFaceHeight * iFaceWidth * sizeof(Double));
    m_fCubeWeight_C = (Double*)malloc((iFaceHeight >> iScaleY) * (iFaceWidth >> iScaleX) * sizeof(Double));
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x < iFaceWidth; x++)
      {
        Double weight = 1.0;
        m_fCubeWeight_Y[iFaceWidth*y+x] = weight;
        fWeightSum_Y += weight;
      }
    }
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x<iFaceWidth; x++)
      {
        m_fCubeWeight_Y[iFaceHeight*y + x] = (m_fCubeWeight_Y[iFaceHeight*y + x]) / fWeightSum_Y / 6.0;
      }
    }

    for (Int y = 0; y < (iFaceHeight >> iScaleY); y++)
    {
      for (Int x = 0; x< (iFaceWidth >> iScaleX); x++)
      {
        Double weight = 1.0;
        m_fCubeWeight_C[(iFaceWidth>>iScaleX)*y+x]=weight;
        fWeightSum_C += weight;
      }
    }

    for (Int y = 0; y < (iFaceHeight >> iScaleY); y++)
    {
      for (Int x = 0; x< (iFaceWidth >> iScaleX); x++)
      {
        m_fCubeWeight_C[(iFaceWidth >> iScaleX)*y + x] = (m_fCubeWeight_C[(iFaceWidth >> iScaleX)*y + x]) / fWeightSum_C / 6.0;
      }
    }
  }
#endif
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
  else if (pcCodingGeomtry->getType() == SVIDEO_EQUIANGULARCUBEMAP)
  {
    Double fWeightSum_Y = 0;
    Double fWeightSum_C = 0;

    m_fCubeWeight_Y = (Double*)malloc(iFaceHeight * iFaceWidth * sizeof(Double));
    m_fCubeWeight_C = (Double*)malloc((iFaceHeight >> iScaleY) * (iFaceWidth >> iScaleX) * sizeof(Double));
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x < iFaceWidth; x++)
      {
        Float pu = (Float)((2.0*(x+0.5))/iFaceWidth-1.0);
        Float pv = (Float)((2.0*(y+0.5))/iFaceHeight-1.0);
       

        Double tu  = stan(pu*S_PI/4.0);
        Double tv  = stan(pv*S_PI/4.0);
        Double weight = 1.0 / (scos(pu*S_PI/4.0)*scos(pu*S_PI/4.0)*scos(pv*S_PI/4.0)*scos(pv*S_PI/4.0) * (tu*tu+tv*tv+1.0) * ssqrt(tu*tu+tv*tv+1.0));

        m_fCubeWeight_Y[iFaceWidth*y+x] = weight;
        fWeightSum_Y += weight;
      }
    }
    for (Int y = 0; y < iFaceHeight; y++)
    {
      for (Int x = 0; x<iFaceWidth; x++)
      {
        m_fCubeWeight_Y[iFaceHeight*y + x] = (m_fCubeWeight_Y[iFaceHeight*y + x]) / fWeightSum_Y / 6.0;
      }
    }
    for (Int y = 0; y < (iFaceHeight >> iScaleY); y++)
    {
      for (Int x = 0; x< (iFaceWidth >> iScaleX); x++)
      {
        Float pu = (Float)((2.0*(x+0.5))/(iFaceWidth>>iScaleX)-1.0);
        Float pv = (Float)((2.0*(y+0.5))/(iFaceHeight>>iScaleY)-1.0);

        Double tu  = stan(pu*S_PI/4.0);
        Double tv  = stan(pv*S_PI/4.0);
        Double weight = 1.0 / (scos(pu*S_PI/4.0)*scos(pu*S_PI/4.0)*scos(pv*S_PI/4.0)*scos(pv*S_PI/4.0) * (tu*tu+tv*tv+1.0) * ssqrt(tu*tu+tv*tv+1.0));
        m_fCubeWeight_C[(iFaceWidth>>iScaleX)*y+x]=weight;
        fWeightSum_C += weight;
      }
    }

    for (Int y = 0; y < (iFaceHeight >> iScaleY); y++)
    {
      for (Int x = 0; x< (iFaceWidth >> iScaleX); x++)
      {
        m_fCubeWeight_C[(iFaceWidth >> iScaleX)*y + x] = (m_fCubeWeight_C[(iFaceWidth >> iScaleX)*y + x]) / fWeightSum_C / 6.0;
      }
    }
  }
#endif
  else
  {
    printf("WS-PSNR does not support for this format: GeoType:%d, FramePackingType:%d!\n", pcCodingGeomtry->getType(), pCodingSVideoInfo->iCompactFPStructure); 
    assert(!"Checking configruation parameters!\n");
  }
}

Void TWSPSNRMetric::xCalculateWSPSNR( TComPicYuv* pcOrgPicYuv, TComPicYuv* pcPicD )
{
  Int iBitDepthForPSNRCalc[MAX_NUM_CHANNEL_TYPE];
  Int iReferenceBitShift[MAX_NUM_CHANNEL_TYPE];
  Int iOutputBitShift[MAX_NUM_CHANNEL_TYPE];
  iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] = std::max(m_outputBitDepth[CHANNEL_TYPE_LUMA], m_referenceBitDepth[CHANNEL_TYPE_LUMA]);
  iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] = std::max(m_outputBitDepth[CHANNEL_TYPE_CHROMA], m_referenceBitDepth[CHANNEL_TYPE_CHROMA]);
  iReferenceBitShift[CHANNEL_TYPE_LUMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] - m_referenceBitDepth[CHANNEL_TYPE_LUMA];
  iReferenceBitShift[CHANNEL_TYPE_CHROMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] - m_referenceBitDepth[CHANNEL_TYPE_CHROMA];
  iOutputBitShift[CHANNEL_TYPE_LUMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] - m_outputBitDepth[CHANNEL_TYPE_LUMA];
  iOutputBitShift[CHANNEL_TYPE_CHROMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] - m_outputBitDepth[CHANNEL_TYPE_CHROMA];

  memset(m_dWSPSNR, 0, sizeof(Double)*3);
  TComPicYuv &picd=*pcPicD;
  //Double SSDspsnr[3]={0, 0 ,0};
  //ChromaFormat chromaFormat = pcPicD->getChromaFormat();

  for(Int chan=0; chan<pcPicD->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel*  pOrg       = pcOrgPicYuv->getAddr(ch);
    const Int   iOrgStride = pcOrgPicYuv->getStride(ch);
    const Pel*  pRec       = picd.getAddr(ch);
    const Int   iRecStride = picd.getStride(ch);
    const Int   iWidth  = pcPicD->getWidth (ch) ;
    const Int   iHeight = pcPicD->getHeight(ch) ;
    Double fWeight =1;
    Double fWeightSum=0;
    //Int   iSize   = iWidth*iHeight;

    Double SSDwpsnr=0;
      
      
    //WS-PSNR
    for(Int y = 0; y < iHeight; y++ )
    {
        
      if (m_codingGeoType==SVIDEO_EQUIRECT)
      {      
        if(!chan)
        {
          fWeight=m_fErpWeight_Y[y];
        }
        else
        {
          fWeight=m_fErpWeight_C[y];
        }
      }
      for(Int x = 0; x < iWidth; x++ )
      {
        Intermediate_Int iDiff = (Intermediate_Int)( (pOrg[x]<<iReferenceBitShift[toChannelType(ch)]) - (pRec[x]<<iOutputBitShift[toChannelType(ch)]) );
        if(  (m_codingGeoType == SVIDEO_CUBEMAP) 
#if SVIDEO_ADJUSTED_CUBEMAP
          || (m_codingGeoType == SVIDEO_ADJUSTEDCUBEMAP)
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL && !SVIDEO_ECP_WSPSNR_FIX_TICKET56
          || (m_codingGeoType == SVIDEO_EQUATORIALCYLINDRICAL)
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
          || (m_codingGeoType == SVIDEO_EQUIANGULARCUBEMAP)
#endif
          )
        {
          if(!chan)
          {
            if(iWidth/4 == iHeight/3 && x >= iWidth/4 && (y< iHeight/3 || y>= 2*iHeight/3))
            {
              fWeight=0;
            }
            else 
            {
              fWeight=m_fCubeWeight_Y[(m_iCodingFaceWidth)*(y%(m_iCodingFaceHeight)) +(x%(m_iCodingFaceWidth))];
            }

          }
          else
          {
            if(iWidth/4 == iHeight/3 && x >= iWidth/4 && (y< iHeight/3 || y>= 2*iHeight/3))
            {
              fWeight=0;
            }
            else
            {
              fWeight=m_fCubeWeight_C[(m_iCodingFaceWidth>>(pcPicD->getComponentScaleX(COMPONENT_Cb)))*(y%(m_iCodingFaceHeight>>(pcPicD->getComponentScaleY(COMPONENT_Cb)))) +(x%(m_iCodingFaceWidth>>(pcPicD->getComponentScaleX(COMPONENT_Cb))))];
            }  
          }
        }
#if SVIDEO_ADJUSTED_EQUALAREA
        else if (m_codingGeoType==SVIDEO_ADJUSTEDEQUALAREA)
#else
        else if (m_codingGeoType==SVIDEO_EQUALAREA)
#endif
        {
          if(!chan)
          {
            fWeight=m_fEapWeight_Y[y*iWidth+x];
          }
          else
          {
            fWeight=m_fEapWeight_C[y*iWidth+x];
          }
        }
        else if (m_codingGeoType==SVIDEO_OCTAHEDRON )
        {
          if(!chan)
          {
            fWeight=m_fOctaWeight_Y[iWidth*y +x];
          }
          else
          {
            fWeight=m_fOctaWeight_C[y*iWidth+x];
          }
        }
        else if ( m_codingGeoType==SVIDEO_ICOSAHEDRON)
        {
          if(!chan)
          {
            fWeight=m_fIcoWeight_Y[iWidth*y +x];
          }
          else
          {
            fWeight=m_fIcoWeight_C[y*iWidth+x];
          }
        }
#if SVIDEO_WSPSNR_SSP
        else if (m_codingGeoType == SVIDEO_SEGMENTEDSPHERE)
        {
            if (!chan)
            {
                fWeight = m_fSspWeight_Y[iWidth*y + x];
            }
            else
            {
                fWeight = m_fSspWeight_C[y*iWidth + x];
            }
        }
#endif
#if SVIDEO_ROTATED_SPHERE
        else if (m_codingGeoType==SVIDEO_ROTATEDSPHERE)
        {
            if(!chan)
            {
                fWeight = m_fRspWeight_Y[iWidth*y + x];
            }
            else
            {
                fWeight = m_fRspWeight_C[y*iWidth + x];
            }
        }
#endif
#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
        else if (m_codingGeoType==SVIDEO_EQUATORIALCYLINDRICAL)
        {
            if(!chan)
            {
                fWeight = m_fEcpWeight_Y[iWidth*y + x];
            }
            else
            {
                fWeight = m_fEcpWeight_C[y*iWidth + x];
            }
        }
#endif
#if SVIDEO_ERP_PADDING
        else if (m_codingGeoType == SVIDEO_EQUIRECT && m_bPERP )
        {
#if 1
            ChromaFormat fmt = pcPicD->getChromaFormat();                
            if ((x < (SVIDEO_ERP_PAD_L >> getComponentScaleX(ch, fmt))) || (x >= (iWidth - (SVIDEO_ERP_PAD_R >> getComponentScaleX(ch, fmt)))))
                fWeight = 0;
            else
                fWeight = (!chan)? m_fErpWeight_Y[y] : m_fErpWeight_C[y];
#else
            if (!chan)
            {
                if( (x < SVIDEO_ERP_PAD_L) || (x >= (iWidth - SVIDEO_ERP_PAD_R)))
                    fWeight = 0;
                else
                    fWeight = m_fErpWeight_Y[y];
            }
            else
            {
                ComponentID chId = ComponentID(chan);
                ChromaFormat fmt = pcPicD->getChromaFormat();
                
                if ((x < (SVIDEO_ERP_PAD_L >> getComponentScaleX(chId, fmt))) || (x >= (iWidth - (SVIDEO_ERP_PAD_R >> getComponentScaleX(chId, fmt)))))
                    fWeight = 0;
                else
                    fWeight = m_fErpWeight_C[y];
            }
#endif
        }
#endif

        if(fWeight>0)
          fWeightSum += fWeight;
        SSDwpsnr   += iDiff * iDiff*fWeight;
      }
    
      pOrg += iOrgStride;
      pRec += iRecStride;

    }
      

    const Int maxval = 255<<(iBitDepthForPSNRCalc[toChannelType(ch)]-8) ;
    //const Double fRefValue = (Double) maxval * maxval * iSize;

    m_dWSPSNR[ch]         = ( SSDwpsnr ? 10.0 * log10( (maxval * maxval*fWeightSum) / (Double)SSDwpsnr ) : 999.99 );
  }
}

#if SVIDEO_WSPSNR_E2E
#if SVIDEO_E2E_METRICS
Void TWSPSNRMetric::setCodingGeoInfo2(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam)
#else
Void TWSPSNRMetric::setCodingGeoInfo2(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, TVideoIOYuv& yuvInputFile, Int iInputWidth, Int iInputHeight, UInt tempSubsampleRatio)
#endif
{
  m_codingGeoType = sRefVideoInfo.geoType; 
  m_iCodingFaceWidth = sRefVideoInfo.iFaceWidth; 
  m_iCodingFaceHeight = sRefVideoInfo.iFaceHeight; 
  m_iChromaSampleLocType = pInGeoParam->iChromaSampleLocType;
#if !SVIDEO_E2E_METRICS
  m_temporalSubsampleRatio = tempSubsampleRatio;
  m_iInputWidth = iInputWidth;
  m_iInputHeight = iInputHeight;
  m_inputChromaFomat = sRefVideoInfo.framePackStruct.chromaFormatIDC;
#endif
  //Rec geometry to reference geometry;
#if !SVIDEO_E2E_METRICS
  if(m_bEnabled)
  {
    m_pcTVideoIOYuvInputFile = &yuvInputFile;
    m_pRefGeometry = TGeometry::create(sRefVideoInfo, pInGeoParam);
    m_pRecGeometry = TGeometry::create(sRecVideoInfo, pInGeoParam);
    m_pcOrgPicYuv = new TComPicYuv;
    m_pcOrgPicYuv->createWithoutCUInfo(iInputWidth, iInputHeight, m_inputChromaFomat, false, S_PAD_MAX, S_PAD_MAX);

    m_pcRecPicYuv = new TComPicYuv;
    m_pcRecPicYuv->createWithoutCUInfo(iInputWidth, iInputHeight, m_inputChromaFomat, true, S_PAD_MAX, S_PAD_MAX);
  }
#endif

}
#if SVIDEO_E2E_METRICS
Void TWSPSNRMetric::xCalculateE2EWSPSNR( TComPicYuv *pcPicYuv, TComPicYuv *pcOrigPicYuv)
#else
Void TWSPSNRMetric::xCalculateE2EWSPSNR( TComPicYuv *pcPicYuv, Int iPOC )
#endif
{
  if(!m_bEnabled)
    return;
#if !SVIDEO_E2E_METRICS
  Int iDeltaFrames = iPOC*m_temporalSubsampleRatio - m_iLastFrmPOC;
  Int aiPad[2]={0,0};
  m_pcTVideoIOYuvInputFile->skipFrames(iDeltaFrames, m_iInputWidth, m_iInputHeight, m_inputChromaFomat);
  m_pcTVideoIOYuvInputFile->read(NULL, m_pcOrgPicYuv, IPCOLOURSPACE_UNCHANGED, aiPad, m_inputChromaFomat, false );
  m_iLastFrmPOC = iPOC*m_temporalSubsampleRatio+1;

  //generate the reconstructed picture in source gemoetry domain;
  if((m_pRecGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRecGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRecGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRecGeometry->compactFramePackConvertYuv(pcPicYuv);
  else
    m_pRecGeometry->convertYuv(pcPicYuv);
#if SVIDEO_ROT_FIX
  m_pRecGeometry->geoConvert(m_pRefGeometry, true);
#else
  m_pRecGeometry->geoConvert(m_pRefGeometry);
#endif
  if((m_pRefGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRefGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRefGeometry->getSVideoInfo()->iCompactFPStructure)
    m_pRefGeometry->compactFramePack(m_pcRecPicYuv);
  else
    m_pRefGeometry->framePack(m_pcRecPicYuv);
#endif
#if SVIDEO_E2E_METRICS
  xCalculateWSPSNR(pcOrigPicYuv, pcPicYuv);
#else
  xCalculateWSPSNR(m_pcOrgPicYuv, m_pcRecPicYuv);
#endif
}
#endif  //SVIDEO_WSPSNR_E2E

#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
Double TWSPSNRMetric::CalculateEcpWeight(SPos& SPosIn, Int iFaceWidth)
{
  // Face 0
  Double weight = 1.0;
  Double b = 0.2;
  Double padmargin = 4.0;
  POSType x = SPosIn.x;
  POSType y = SPosIn.y;
  POSType z = SPosIn.z;
  Double len = ssqrt(x*x + y*y + z*z);
  Double yaw = (POSType)(satan2(-z, x));
  Double pitch = (POSType)(sasin(y / len));

  Double n = (S_PI_2 - sasin(2.0/3.0));
  Double u, v;
  u = ssin(yaw + S_PI_2/2.0)*(S_PI_2 - pitch)/n;
  v = scos(yaw + S_PI_2/2.0)*(S_PI_2 - pitch)/n;

  Double C = u*u + v*v - 4.0*u*u*v*v;
  Double B = ssqrt((u*u + v*v)*C);
  Double A = ssqrt(u*u + v*v - B);
  Double sgn = (u*v >= 0.0) ? 1.0 : -1.0;
  Double w = sgn*A/ssqrt(2.0);

  Double xp = w/v;
  Double yp = w/u;
  Double D;
  D = (-yp - 1.0)/b;
  Double xpp = 1.0 + tanh(D);

  Double dwdu = sgn*(2.0*u - (2.0*u*C + (2.0*u - 8.0*u*v*v)*(u*u + v*v))/(2.0*B))/(2.0*ssqrt(2.0)*A);
  Double dwdv = sgn*(2.0*v - (2.0*v*C + (2.0*v - 8.0*v*u*u)*(u*u + v*v))/(2.0*B))/(2.0*ssqrt(2.0)*A);

  Double dxpdu = dwdu/v;
  Double dxpdv = (dwdv*v - w)/(v*v);
  Double dypdu = (dwdu*u - w)/(u*u);
  Double dypdv = dwdv/u;
  Double dxppdyp;
  dxppdyp = 1.0/(-b*cosh(D)*cosh(D));
  Double dxdxp = xpp/(satan(xpp)*(1 + xpp*xpp*xp*xp));
  Double dxdxpp = (satan(xpp)*xp/(1 + xpp*xpp*xp*xp) - satan(xpp*xp)/(1 + xpp*xpp))/(satan(xpp)*satan(xpp));

  Double padfactor = 1.0 + 2.0*padmargin/((Double)iFaceWidth - 2.0*padmargin);
  Double apadfactor = (Double)iFaceWidth/((Double)iFaceWidth - padmargin);
  Double dxpppdx = 1.0/padfactor;
  Double dypppdy = 1.0/apadfactor;

  Double phi = yaw, theta = pitch;
  Double dudtheta, dudphi, dvdtheta, dvdphi;
  dudtheta = -ssin(phi + S_PI_2/2.0)/n;
  dudphi = (S_PI_2 - theta)*scos(phi + S_PI_2/2.0)/n;
  dvdtheta = -scos(phi + S_PI_2/2.0)/n;
  dvdphi = -(S_PI_2 - theta)*ssin(phi + S_PI_2/2.0)/n;

  Double dxpdphi = (dxpdu*dudphi + dxpdv*dvdphi);
  Double dxppdphi = (dxppdyp*(dypdu*dudphi + dypdv*dvdphi));
  Double dxpppdphi = (dxpppdx*(dxdxp*dxpdphi + dxdxpp*dxppdphi));

  Double dxpdtheta = (dxpdu*dudtheta + dxpdv*dvdtheta);
  Double dxppdtheta = (dxppdyp*(dypdu*dudtheta + dypdv*dvdtheta));
  Double dxpppdtheta = (dxpppdx*(dxdxp*dxpdtheta + dxdxpp*dxppdtheta));

  Double dypppdphi = (dypppdy*(dypdu*dudphi + dypdv*dvdphi));
  Double dypppdtheta = (dypppdy*(dypdu*dudtheta + dypdv*dvdtheta));
  Double J = (dxpppdphi*dypppdtheta - dxpppdtheta*dypppdphi);

  weight = scos(theta)/fabs(J);
  assert(weight >= 0.0);
  return weight;
}
#endif
#endif
