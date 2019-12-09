/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

/** \file     TEncRateCtrl.cpp
    \brief    Rate control manager class
*/
#include "TEncRateCtrl.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComChromaFormat.h"

#if CSVT_2018
#include "TEncTop.h"
#endif

#include <cmath>

using namespace std;

//sequence level
TEncRCSeq::TEncRCSeq()
{
  m_totalFrames         = 0;
  m_targetRate          = 0;
  m_frameRate           = 0;
  m_targetBits          = 0;
  m_GOPSize             = 0;
  m_picWidth            = 0;
  m_picHeight           = 0;
  m_LCUWidth            = 0;
  m_LCUHeight           = 0;
  m_numberOfLevel       = 0;
  m_numberOfLCU         = 0;
#if BU_LEVEL_RATE_CONTROL
  m_numberOfBU          = 0;
  m_numberOfLCUInBU     = 0;
#endif
  m_averageBits         = 0;
  m_bitsRatio           = NULL;
  m_GOPID2Level         = NULL;
  m_picPara             = NULL;
  m_LCUPara             = NULL;
#if BU_LEVEL_RATE_CONTROL
  m_BUPara              = NULL;
#endif
#if CTU_LEVEL_BIT_ALLOCATION || VCIP_2017
  m_ctuWeight           = NULL;
#endif
#if BU_LEVEL_RATE_CONTROL
  m_BUWeight            = NULL;
#endif
  m_numberOfPixel       = 0;
  m_framesLeft          = 0;
  m_bitsLeft            = 0;
  m_useLCUSeparateModel = false;
  m_adaptiveBit         = 0;
  m_lastLambda          = 0.0;
#if ERP_RC
  m_bitDepth            = 0;
#endif
}

TEncRCSeq::~TEncRCSeq()
{
  destroy();
}

Void TEncRCSeq::create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, Bool useLCUSeparateModel, Int adaptiveBit )
{
  destroy();
  m_totalFrames         = totalFrames;
  m_targetRate          = targetBitrate;
  m_frameRate           = frameRate;
  m_GOPSize             = GOPSize;
  m_picWidth            = picWidth;
  m_picHeight           = picHeight;
  m_LCUWidth            = LCUWidth;
  m_LCUHeight           = LCUHeight;
  m_numberOfLevel       = numberOfLevel;
  m_useLCUSeparateModel = useLCUSeparateModel;

  m_numberOfPixel   = m_picWidth * m_picHeight;
  m_targetBits      = (Int64)m_totalFrames * (Int64)m_targetRate / (Int64)m_frameRate;
  m_seqTargetBpp = (Double)m_targetRate / (Double)m_frameRate / (Double)m_numberOfPixel;
  if ( m_seqTargetBpp < 0.03 )
  {
    m_alphaUpdate = 0.01;
    m_betaUpdate  = 0.005;
  }
  else if ( m_seqTargetBpp < 0.08 )
  {
    m_alphaUpdate = 0.05;
    m_betaUpdate  = 0.025;
  }
  else if ( m_seqTargetBpp < 0.2 )
  {
    m_alphaUpdate = 0.1;
    m_betaUpdate  = 0.05;
  }
  else if ( m_seqTargetBpp < 0.5 )
  {
    m_alphaUpdate = 0.2;
    m_betaUpdate  = 0.1;
  }
  else
  {
    m_alphaUpdate = 0.4;
    m_betaUpdate  = 0.2;
  }

  m_averageBits     = (Int)(m_targetBits / totalFrames);
  Int picWidthInBU  = ( m_picWidth  % m_LCUWidth  ) == 0 ? m_picWidth  / m_LCUWidth  : m_picWidth  / m_LCUWidth  + 1;
  Int picHeightInBU = ( m_picHeight % m_LCUHeight ) == 0 ? m_picHeight / m_LCUHeight : m_picHeight / m_LCUHeight + 1;
  m_numberOfLCU     = picWidthInBU * picHeightInBU;

#if BU_LEVEL_RATE_CONTROL
  m_numberOfBU = picHeightInBU;
  m_numberOfLCUInBU = picWidthInBU;
#endif

  m_bitsRatio   = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_bitsRatio[i] = 1;
  }

  m_GOPID2Level = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_GOPID2Level[i] = 1;
  }

  m_picPara = new TRCParameter[m_numberOfLevel];
  for ( Int i=0; i<m_numberOfLevel; i++ )
  {
    m_picPara[i].m_alpha = 0.0;
    m_picPara[i].m_beta  = 0.0;
#if ERP_RC
    m_picPara[i].m_validPix = -1;
#endif
  }

#if CTU_LEVEL_BIT_ALLOCATION || VCIP_2017
  m_ctuWeight = new Double[m_numberOfLCU];
  for (Int i = 0; i < m_numberOfLCU; i++ )
  {
    m_ctuWeight[i] = 0.0;
  }
#endif

#if BU_LEVEL_RATE_CONTROL
  m_BUWeight = new Double[m_numberOfBU];
  for (Int i = 0; i < m_numberOfBU; i++)
  {
    m_BUWeight[i] = 0.0;
  }
#endif

#if BU_LEVEL_RATE_CONTROL
  m_BUPara = new TRCParameter*[m_numberOfLevel];
  for (Int i = 0; i < m_numberOfLevel; i++ )
  {
    m_BUPara[i] = new TRCParameter[m_numberOfBU];
    for (Int j = 0; j < m_numberOfBU; j++ )
    {
      m_BUPara[i][j].m_alpha = 0.0;
      m_BUPara[i][j].m_beta = 0.0;
#if ERP_RC
      m_BUPara[i][j].m_validPix = -1;
#endif
    }
  }
#endif

  if ( m_useLCUSeparateModel )
  {
    m_LCUPara = new TRCParameter*[m_numberOfLevel];
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      m_LCUPara[i] = new TRCParameter[m_numberOfLCU];
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j].m_alpha = 0.0;
        m_LCUPara[i][j].m_beta  = 0.0;
#if ERP_RC
        m_LCUPara[i][j].m_validPix = -1;
#endif
      }
    }
  }

  m_framesLeft = m_totalFrames;
  m_bitsLeft   = m_targetBits;
  m_adaptiveBit = adaptiveBit;
  m_lastLambda = 0.0;
}

Void TEncRCSeq::destroy()
{
  if (m_bitsRatio != NULL)
  {
    delete[] m_bitsRatio;
    m_bitsRatio = NULL;
  }

  if ( m_GOPID2Level != NULL )
  {
    delete[] m_GOPID2Level;
    m_GOPID2Level = NULL;
  }

  if ( m_picPara != NULL )
  {
    delete[] m_picPara;
    m_picPara = NULL;
  }

#if CTU_LEVEL_BIT_ALLOCATION || VCIP_2017
  if ( m_ctuWeight != NULL )
  {
    delete[] m_ctuWeight;
    m_ctuWeight = NULL;
  }
#endif

#if BU_LEVEL_RATE_CONTROL
  if ( m_BUWeight != NULL )
  {
    delete[] m_BUWeight;
    m_BUWeight = NULL;
  }
#endif

#if BU_LEVEL_RATE_CONTROL
  if ( m_BUPara != NULL )
  {
    for (Int i = 0; i < m_numberOfLevel; i++ )
    {
      delete[] m_BUPara[i];
    }
    delete[] m_BUPara;
    m_BUPara = NULL;
  }
#endif

  if ( m_LCUPara != NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      delete[] m_LCUPara[i];
    }
    delete[] m_LCUPara;
    m_LCUPara = NULL;
  }
}

Void TEncRCSeq::initBitsRatio( Int bitsRatio[])
{
  for (Int i=0; i<m_GOPSize; i++)
  {
    m_bitsRatio[i] = bitsRatio[i];
  }
}

Void TEncRCSeq::initGOPID2Level( Int GOPID2Level[] )
{
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_GOPID2Level[i] = GOPID2Level[i];
  }
}

#if BU_LEVEL_RATE_CONTROL
Void TEncRCSeq::initBUWeight()
{
  const Double PI = 3.1416;

  for (Int BUIdx = 0; BUIdx < m_numberOfBU; BUIdx++)
  {
    Int yStartCoor = BUIdx * m_LCUHeight;
    Int yEndCoor = yStartCoor + m_LCUHeight - 1 > m_picHeight - 1 ? m_picHeight - 1 : yStartCoor + m_LCUHeight - 1;

    Double sumWeight = 0.0;
    for (Int yCoor = yStartCoor; yCoor <= yEndCoor; yCoor++ )
    {
      sumWeight += cos( ( yCoor - m_picHeight / 2 + 1.0 / 2 ) * PI / m_picHeight );
    }
    Double averageWeight = sumWeight / (yEndCoor - yStartCoor + 1);
    m_BUWeight[BUIdx] = averageWeight;
  }
}
#endif

#if CTU_LEVEL_BIT_ALLOCATION || VCIP_2017
Void TEncRCSeq::initCtuWeight()
{
  Int picWidthInBU = (m_picWidth  % m_LCUWidth) == 0 ? m_picWidth / m_LCUWidth : m_picWidth / m_LCUWidth + 1;
  const Double PI = 3.1416;

  for (Int ctuIdx = 0; ctuIdx < m_numberOfLCU; ctuIdx++ )
  {
    if ( ctuIdx % picWidthInBU == 0 )
    {
      Int yStartCoor = ctuIdx / picWidthInBU * m_LCUHeight;
      Int yEndCoor = yStartCoor + m_LCUHeight - 1 > m_picHeight - 1 ? m_picHeight - 1 : yStartCoor + m_LCUHeight - 1;

      Double sumWeight = 0.0;
      for (Int yCoor = yStartCoor; yCoor <= yEndCoor; yCoor++ )
      {
        sumWeight += cos( ( yCoor - m_picHeight / 2 + 1.0 / 2 ) * PI / m_picHeight );
      }
      Double averageWeight = sumWeight / (yEndCoor - yStartCoor + 1);
      m_ctuWeight[ctuIdx] = averageWeight;
    }
    else
    {
      m_ctuWeight[ctuIdx] = m_ctuWeight[ctuIdx - 1];
    }
  }
}
#endif

Void TEncRCSeq::initPicPara( TRCParameter* picPara )
{
  assert( m_picPara != NULL );

  if ( picPara == NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      if (i>0)
      {
        m_picPara[i].m_alpha = 3.2003;
        m_picPara[i].m_beta  = -1.367;
      }
      else
      {
        m_picPara[i].m_alpha = ALPHA;
        m_picPara[i].m_beta  = BETA2;
      }
    }
  }
  else
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      m_picPara[i] = picPara[i];
    }
  }
}

Void TEncRCSeq::initLCUPara( TRCParameter** LCUPara )
{
  if ( m_LCUPara == NULL )
  {
    return;
  }
  if ( LCUPara == NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j].m_alpha = m_picPara[i].m_alpha;
        m_LCUPara[i][j].m_beta  = m_picPara[i].m_beta;
      }
    }
  }
  else
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j] = LCUPara[i][j];
      }
    }
  }
}

#if BU_LEVEL_RATE_CONTROL
Void TEncRCSeq::initBUPara(TRCParameter** BUPara)
{
  if (BUPara == NULL)
  {
    for (Int i = 0; i < m_numberOfLevel; i++)
    {
      for (Int j = 0; j < m_numberOfBU; j++)
      {
        m_BUPara[i][j].m_alpha = m_picPara[i].m_alpha;
        m_BUPara[i][j].m_beta = m_picPara[i].m_beta;
      }
    }
  }
  else
  {
    for (Int i = 0; i < m_numberOfLevel; i++)
    {
      for (Int j = 0; j < m_numberOfBU; j++)
      {
        m_BUPara[i][j] = BUPara[i][j];
      }
    }
  }
}
#endif

Void TEncRCSeq::updateAfterPic ( Int bits )
{
  m_bitsLeft -= bits;
  m_framesLeft--;
}

Void TEncRCSeq::setAllBitRatio( Double basicLambda, Double* equaCoeffA, Double* equaCoeffB )
{
  Int* bitsRatio = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
#if ERP_RC
    bitsRatio[i] = (Int)( equaCoeffA[i] * pow( basicLambda, equaCoeffB[i] ) * (Double)getPicPara(getGOPID2Level(i)).m_validPix );
#else
    bitsRatio[i] = (Int)( equaCoeffA[i] * pow( basicLambda, equaCoeffB[i] ) * m_numberOfPixel );
#endif
  }
  initBitsRatio( bitsRatio );
  delete[] bitsRatio;
}

//GOP level
TEncRCGOP::TEncRCGOP()
{
  m_encRCSeq  = NULL;
  m_picTargetBitInGOP = NULL;
  m_numPic     = 0;
  m_targetBits = 0;
  m_picLeft    = 0;
  m_bitsLeft   = 0;
}

TEncRCGOP::~TEncRCGOP()
{
  destroy();
}

Void TEncRCGOP::create( TEncRCSeq* encRCSeq, Int numPic )
{
  destroy();
  Int targetBits = xEstGOPTargetBits( encRCSeq, numPic );

  printf("the GOP target bits are %d\n", targetBits);

  if ( encRCSeq->getAdaptiveBits() > 0 && encRCSeq->getLastLambda() > 0.1 )
  {
    Double targetBpp = (Double)targetBits / encRCSeq->getNumPixel();
    Double basicLambda = 0.0;
    Double* lambdaRatio = new Double[encRCSeq->getGOPSize()];
    Double* equaCoeffA = new Double[encRCSeq->getGOPSize()];
    Double* equaCoeffB = new Double[encRCSeq->getGOPSize()];

    if ( encRCSeq->getAdaptiveBits() == 1 )   // for GOP size =4, low delay case
    {
      if ( encRCSeq->getLastLambda() < 120.0 )
      {
        lambdaRatio[1] = 0.725 * log( encRCSeq->getLastLambda() ) + 0.5793;
        lambdaRatio[0] = 1.3 * lambdaRatio[1];
        lambdaRatio[2] = 1.3 * lambdaRatio[1];
        lambdaRatio[3] = 1.0;
      }
      else
      {
        lambdaRatio[0] = 5.0;
        lambdaRatio[1] = 4.0;
        lambdaRatio[2] = 5.0;
        lambdaRatio[3] = 1.0;
      }
    }
    else if ( encRCSeq->getAdaptiveBits() == 2 )  // for GOP size = 8, random access case
    {
      if ( encRCSeq->getLastLambda() < 90.0 )
      {
        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = 0.725 * log( encRCSeq->getLastLambda() ) + 0.7963;
        lambdaRatio[2] = 1.3 * lambdaRatio[1];
        lambdaRatio[3] = 3.25 * lambdaRatio[1];
        lambdaRatio[4] = 3.25 * lambdaRatio[1];
        lambdaRatio[5] = 1.3  * lambdaRatio[1];
        lambdaRatio[6] = 3.25 * lambdaRatio[1];
        lambdaRatio[7] = 3.25 * lambdaRatio[1];
      }
      else
      {
        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = 4.0;
        lambdaRatio[2] = 5.0;
        lambdaRatio[3] = 12.3;
        lambdaRatio[4] = 12.3;
        lambdaRatio[5] = 5.0;
        lambdaRatio[6] = 12.3;
        lambdaRatio[7] = 12.3;
      }
    }
#if ERP_RC
    else if (encRCSeq->getAdaptiveBits() == 3)  // for GOP size = 16, random access case
    {
      {
        Int bitdepth_luma_scale = 0;

        double hierarQp = 4.2005 * log(encRCSeq->getLastLambda() / pow(2.0, bitdepth_luma_scale)) + 13.7122;     // GOP bit
        double qpLev2 = (hierarQp + 0.0) + 0.2016  * (hierarQp + 0.0) - 4.8848;
        double qpLev3 = (hierarQp + 3.0) + 0.22286 * (hierarQp + 3.0) - 5.7476;
        double qpLev4 = (hierarQp + 4.0) + 0.2333  * (hierarQp + 4.0) - 5.9;
        double qpLev5 = (hierarQp + 5.0) + 0.3     * (hierarQp + 5.0) - 7.1444;

        double lambdaLev1 = exp((hierarQp - 13.7122) / 4.2005) *pow(2.0, bitdepth_luma_scale);
        double lambdaLev2 = exp((qpLev2 - 13.7122) / 4.2005) * pow(2.0, bitdepth_luma_scale);
        double lambdaLev3 = exp((qpLev3 - 13.7122) / 4.2005) * pow(2.0, bitdepth_luma_scale);
        double lambdaLev4 = exp((qpLev4 - 13.7122) / 4.2005) * pow(2.0, bitdepth_luma_scale);
        double lambdaLev5 = exp((qpLev5 - 13.7122) / 4.2005) * pow(2.0, bitdepth_luma_scale);

        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = lambdaLev2 / lambdaLev1;
        lambdaRatio[2] = lambdaLev3 / lambdaLev1;
        lambdaRatio[3] = lambdaLev4 / lambdaLev1;
        lambdaRatio[4] = lambdaLev5 / lambdaLev1;
        lambdaRatio[5] = lambdaLev5 / lambdaLev1;
        lambdaRatio[6] = lambdaLev4 / lambdaLev1;
        lambdaRatio[7] = lambdaLev5 / lambdaLev1;
        lambdaRatio[8] = lambdaLev5 / lambdaLev1;
        lambdaRatio[9] = lambdaLev3 / lambdaLev1;
        lambdaRatio[10] = lambdaLev4 / lambdaLev1;
        lambdaRatio[11] = lambdaLev5 / lambdaLev1;
        lambdaRatio[12] = lambdaLev5 / lambdaLev1;
        lambdaRatio[13] = lambdaLev4 / lambdaLev1;
        lambdaRatio[14] = lambdaLev5 / lambdaLev1;
        lambdaRatio[15] = lambdaLev5 / lambdaLev1;
      }
    }
#endif

    xCalEquaCoeff( encRCSeq, lambdaRatio, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );

#if ERP_RC
    basicLambda = xSolveEqua( encRCSeq, targetBpp, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );
#else
    basicLambda = xSolveEqua( targetBpp, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );
#endif
    encRCSeq->setAllBitRatio( basicLambda, equaCoeffA, equaCoeffB );

    delete []lambdaRatio;
    delete []equaCoeffA;
    delete []equaCoeffB;
  }

  m_picTargetBitInGOP = new Int[numPic];
  Int i;
  Int totalPicRatio = 0;
  Int currPicRatio = 0;
  for ( i=0; i<numPic; i++ )
  {
    totalPicRatio += encRCSeq->getBitRatio( i );
  }
  for ( i=0; i<numPic; i++ )
  {
    currPicRatio = encRCSeq->getBitRatio( i );
    m_picTargetBitInGOP[i] = (Int)( ((Double)targetBits) * currPicRatio / totalPicRatio );
  }

  m_encRCSeq    = encRCSeq;
  m_numPic       = numPic;
  m_targetBits   = targetBits;
  m_picLeft      = m_numPic;
  m_bitsLeft     = m_targetBits;
}

Void TEncRCGOP::xCalEquaCoeff( TEncRCSeq* encRCSeq, Double* lambdaRatio, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize )
{
  for ( Int i=0; i<GOPSize; i++ )
  {
    Int frameLevel = encRCSeq->getGOPID2Level(i);
#if CSVT_2018
    Double alpha   = 0.0;
    Double beta    = 0.0;
    if(MyAlgorithmPara::Framelevel<=0)
    {
      alpha = encRCSeq->getPicPara(frameLevel).m_alpha;
      beta = encRCSeq->getPicPara(frameLevel).m_beta;
    }
    else
    {
      alpha = -MyAlgorithmPara::PicParaC[frameLevel] * MyAlgorithmPara::PicParaK[frameLevel];
      beta = MyAlgorithmPara::PicParaK[frameLevel] - 1.0;
    }
#else
    Double alpha   = encRCSeq->getPicPara(frameLevel).m_alpha;
    Double beta    = encRCSeq->getPicPara(frameLevel).m_beta;
#endif
    equaCoeffA[i] = pow( 1.0/alpha, 1.0/beta ) * pow( lambdaRatio[i], 1.0/beta );
    equaCoeffB[i] = 1.0/beta;
  }
}

#if ERP_RC
Double TEncRCGOP::xSolveEqua(TEncRCSeq* encRCSeq, Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize)
#else
Double TEncRCGOP::xSolveEqua( Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize )
#endif
{
  Double solution = 100.0;
  Double minNumber = 0.1;
  Double maxNumber = 10000.0;
  for ( Int i=0; i<g_RCIterationNum; i++ )
  {
    Double fx = 0.0;
    for ( Int j=0; j<GOPSize; j++ )
    {
#if ERP_RC
      double tmpBpp = equaCoeffA[j] * pow(solution, equaCoeffB[j]);
      double actualBpp = tmpBpp * (double)encRCSeq->getPicPara(encRCSeq->getGOPID2Level(j)).m_validPix / (double)encRCSeq->getNumPixel();

      fx += actualBpp;
#else
      fx += equaCoeffA[j] * pow( solution, equaCoeffB[j] );
#endif
    }

    if ( fabs( fx - targetBpp ) < 0.000001 )
    {
      break;
    }

    if ( fx > targetBpp )
    {
      minNumber = solution;
      solution = ( solution + maxNumber ) / 2.0;
    }
    else
    {
      maxNumber = solution;
      solution = ( solution + minNumber ) / 2.0;
    }
  }

  solution = Clip3( 0.1, 10000.0, solution );
  return solution;
}

Void TEncRCGOP::destroy()
{
  m_encRCSeq = NULL;
  if ( m_picTargetBitInGOP != NULL )
  {
    delete[] m_picTargetBitInGOP;
    m_picTargetBitInGOP = NULL;
  }
}

Void TEncRCGOP::updateAfterPicture( Int bitsCost )
{
  m_bitsLeft -= bitsCost;
  m_picLeft--;
}

Int TEncRCGOP::xEstGOPTargetBits( TEncRCSeq* encRCSeq, Int GOPSize )
{
#if CSVT_2018
  Int realInfluencePicture;
  if (MyAlgorithmPara::frameallocation > 0)
  {
    realInfluencePicture = min(encRCSeq->getTotalFrames() - 100, encRCSeq->getFramesLeft());
  }
  else
  {
    realInfluencePicture = min(g_RCSmoothWindowSize, encRCSeq->getFramesLeft());
  }
#else
  Int realInfluencePicture = min(g_RCSmoothWindowSize, encRCSeq->getFramesLeft());
#endif

  Int averageTargetBitsPerPic = (Int)( encRCSeq->getTargetBits() / encRCSeq->getTotalFrames() );
  Int currentTargetBitsPerPic = (Int)( ( encRCSeq->getBitsLeft() - averageTargetBitsPerPic * (encRCSeq->getFramesLeft() - realInfluencePicture) ) / realInfluencePicture );
  Int targetBits = currentTargetBitsPerPic * GOPSize;

  if ( targetBits < 200 )
  {
    targetBits = 200;   // at least allocate 200 bits for one GOP
  }

  return targetBits;
}

//picture level
TEncRCPic::TEncRCPic()
{
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;

  m_frameLevel    = 0;
  m_numberOfPixel = 0;
  m_numberOfLCU   = 0;
#if BU_LEVEL_RATE_CONTROL
  m_numberOfBU    = 0;
  m_numberOfLCUInBU = 0;
#endif
  m_targetBits    = 0;
  m_estHeaderBits = 0;
  m_estPicQP      = 0;
  m_estPicLambda  = 0.0;

  m_LCULeft       = 0;
#if BU_LEVEL_RATE_CONTROL
  m_BULeft        = 0;
  m_LCULeftInBU   = 0;
#endif
  m_bitsLeft      = 0;
  m_pixelsLeft    = 0;

  m_LCUs         = NULL;
#if BU_LEVEL_RATE_CONTROL
  m_BUs          = NULL;
#endif
  m_picActualHeaderBits = 0;
  m_picActualBits       = 0;
  m_picQP               = 0;
  m_picLambda           = 0.0;
#if ERP_RC
  m_picMSE           = 0.0;
  m_validPixelsInPic = 0;
#endif
}

TEncRCPic::~TEncRCPic()
{
  destroy();
}

Int TEncRCPic::xEstPicTargetBits( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP )
{
  Int targetBits        = 0;
  Int GOPbitsLeft       = encRCGOP->getBitsLeft();

  Int i;
  Int currPicPosition = encRCGOP->getNumPic()-encRCGOP->getPicLeft();
  Int currPicRatio    = encRCSeq->getBitRatio( currPicPosition );
  Int totalPicRatio   = 0;
  for ( i=currPicPosition; i<encRCGOP->getNumPic(); i++ )
  {
    totalPicRatio += encRCSeq->getBitRatio( i );
  }

  targetBits  = Int( ((Double)GOPbitsLeft) * currPicRatio / totalPicRatio );

  if ( targetBits < 100 )
  {
    targetBits = 100;   // at least allocate 100 bits for one picture
  }

  if ( m_encRCSeq->getFramesLeft() > 16 )
  {
    targetBits = Int( g_RCWeightPicRargetBitInBuffer * targetBits + g_RCWeightPicTargetBitInGOP * m_encRCGOP->getTargetBitInGOP( currPicPosition ) );
  }

  printf("the picture target bits are %d\n", targetBits);

  return targetBits;
}

Int TEncRCPic::xEstPicHeaderBits( list<TEncRCPic*>& listPreviousPictures, Int frameLevel )
{
  Int numPreviousPics   = 0;
  Int totalPreviousBits = 0;

  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == frameLevel )
    {
      totalPreviousBits += (*it)->getPicActualHeaderBits();
      numPreviousPics++;
    }
  }

  Int estHeaderBits = 0;
  if ( numPreviousPics > 0 )
  {
    estHeaderBits = totalPreviousBits / numPreviousPics;
  }

  return estHeaderBits;
}

Int TEncRCPic::xEstPicLowerBound(TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP)
{
  Int lowerBound = 0;
  Int GOPbitsLeft = encRCGOP->getBitsLeft();

  const Int nextPicPosition = (encRCGOP->getNumPic() - encRCGOP->getPicLeft() + 1) % encRCGOP->getNumPic();
  const Int nextPicRatio = encRCSeq->getBitRatio(nextPicPosition);

  Int totalPicRatio = 0;
  for (Int i = nextPicPosition; i < encRCGOP->getNumPic(); i++)
  {
    totalPicRatio += encRCSeq->getBitRatio(i);
  }

  if (nextPicPosition == 0)
  {
    GOPbitsLeft = encRCGOP->getTargetBits();
  }
  else
  {
    GOPbitsLeft -= m_targetBits;
  }

  lowerBound = Int(((Double)GOPbitsLeft) * nextPicRatio / totalPicRatio);

  if (lowerBound < 100)
  {
    lowerBound = 100;   // at least allocate 100 bits for one picture
  }

  if (m_encRCSeq->getFramesLeft() > 16)
  {
    lowerBound = Int(g_RCWeightPicRargetBitInBuffer * lowerBound + g_RCWeightPicTargetBitInGOP * m_encRCGOP->getTargetBitInGOP(nextPicPosition));
  }

  return lowerBound;
}

Void TEncRCPic::addToPictureLsit( list<TEncRCPic*>& listPreviousPictures )
{
  if ( listPreviousPictures.size() > g_RCMaxPicListSize )
  {
    TEncRCPic* p = listPreviousPictures.front();
    listPreviousPictures.pop_front();
    p->destroy();
    delete p;
  }

  listPreviousPictures.push_back( this );
}

Void TEncRCPic::create( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP, Int frameLevel, list<TEncRCPic*>& listPreviousPictures )
{
  destroy();
  m_encRCSeq = encRCSeq;
  m_encRCGOP = encRCGOP;

  Int targetBits    = xEstPicTargetBits( encRCSeq, encRCGOP );
  Int estHeaderBits = xEstPicHeaderBits( listPreviousPictures, frameLevel );

  if ( targetBits < estHeaderBits + 100 )
  {
    targetBits = estHeaderBits + 100;   // at least allocate 100 bits for picture data
  }

  m_frameLevel       = frameLevel;
  m_numberOfPixel    = encRCSeq->getNumPixel();
  m_numberOfLCU      = encRCSeq->getNumberOfLCU();
#if BU_LEVEL_RATE_CONTROL
  m_numberOfBU       = encRCSeq->getNumberofBU();
  m_numberOfLCUInBU  = encRCSeq->getNumberofCTUInBU();
#endif
  m_estPicLambda     = 100.0;
  m_targetBits       = targetBits;
  m_estHeaderBits    = estHeaderBits;
  m_bitsLeft         = m_targetBits;
  Int picWidth       = encRCSeq->getPicWidth();
  Int picHeight      = encRCSeq->getPicHeight();
  Int LCUWidth       = encRCSeq->getLCUWidth();
  Int LCUHeight      = encRCSeq->getLCUHeight();
  Int picWidthInLCU  = ( picWidth  % LCUWidth  ) == 0 ? picWidth  / LCUWidth  : picWidth  / LCUWidth  + 1;
  Int picHeightInLCU = ( picHeight % LCUHeight ) == 0 ? picHeight / LCUHeight : picHeight / LCUHeight + 1;
  m_lowerBound       = xEstPicLowerBound( encRCSeq, encRCGOP );

  m_LCULeft         = m_numberOfLCU;
#if BU_LEVEL_RATE_CONTROL
  m_BULeft          = m_numberOfBU;
  m_LCULeftInBU     = m_numberOfLCUInBU;
#endif
  m_bitsLeft       -= m_estHeaderBits;
  m_pixelsLeft      = m_numberOfPixel;
  m_LCUs           = new TRCLCU[m_numberOfLCU];
#if BU_LEVEL_RATE_CONTROL
  m_BUs            = new TRCBU[m_numberOfBU];
#endif

  Int i, j;
  Int LCUIdx;
#if BU_LEVEL_RATE_CONTROL
  Int BUIdx;
#endif
  for ( i=0; i<picWidthInLCU; i++ )
  {
    for ( j=0; j<picHeightInLCU; j++ )
    {
      LCUIdx = j*picWidthInLCU + i;
      m_LCUs[LCUIdx].m_actualBits = 0;
#if ERP_RC
      m_LCUs[LCUIdx].m_actualSSE = 0.0;
      m_LCUs[LCUIdx].m_actualMSE = 0.0;
#endif
      m_LCUs[LCUIdx].m_QP         = 0;
      m_LCUs[LCUIdx].m_lambda     = 0.0;
      m_LCUs[LCUIdx].m_targetBits = 0;
      m_LCUs[LCUIdx].m_bitWeight  = 1.0;
      Int currWidth  = ( (i == picWidthInLCU -1) ? picWidth  - LCUWidth *(picWidthInLCU -1) : LCUWidth  );
      Int currHeight = ( (j == picHeightInLCU-1) ? picHeight - LCUHeight*(picHeightInLCU-1) : LCUHeight );
      m_LCUs[LCUIdx].m_numberOfPixel = currWidth * currHeight;

#if BU_LEVEL_RATE_CONTROL
      if ( i==0 )
      {
        BUIdx = j;
        m_BUs[BUIdx].m_actualBits = 0;
#if ERP_RC
        m_BUs[BUIdx].m_actualSSE = 0.0;
        m_BUs[BUIdx].m_actualMSE = 0.0;
#endif
        m_BUs[BUIdx].m_estimateQP = 0;
        m_BUs[BUIdx].m_actualQP = 0;
        m_BUs[BUIdx].m_estimateLambda = 0.0;                                       // For multiplication
        m_BUs[BUIdx].m_actualLambda = 0.0;

        m_BUs[BUIdx].m_targetBits = 0;
        m_BUs[BUIdx].m_bitWeight = 1.0;
        m_BUs[BUIdx].m_numberOfPixel = picWidth * currHeight;              // The actual pixels in each BU
      }
#endif
    }
  }
  m_picActualHeaderBits = 0;
  m_picActualBits       = 0;
  m_picQP               = 0;
  m_picLambda           = 0.0;
#if ERP_RC
  m_picMSE = 0.0;
  m_validPixelsInPic = 0;
#endif
}

Void TEncRCPic::destroy()
{
  if( m_LCUs != NULL )
  {
    delete[] m_LCUs;
    m_LCUs = NULL;
  }
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;
}

Double TEncRCPic::estimatePicLambda( list<TEncRCPic*>& listPreviousPictures, SliceType eSliceType)
{

#if CSVT_2018
  Double alpha         = 0.0;
  Double beta          = 0.0;
  /*Lisx*/
  if (MyAlgorithmPara::Framelevel <= 0)
  {
    alpha = m_encRCSeq->getPicPara(m_frameLevel).m_alpha;
    beta = m_encRCSeq->getPicPara(m_frameLevel).m_beta;
  }
  else
  {
    alpha = -MyAlgorithmPara::PicParaC[m_frameLevel] * MyAlgorithmPara::PicParaK[m_frameLevel];
    beta = MyAlgorithmPara::PicParaK[m_frameLevel] - 1.0;
  }
#else
  Double alpha         = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double beta          = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
#endif
  
  
  Double bpp       = (Double)m_targetBits/(Double)m_numberOfPixel;

#if ERP_RC
  Int lastPicValPix = 0;
  if (listPreviousPictures.size() > 0)
  {
    lastPicValPix = m_encRCSeq->getPicPara(m_frameLevel).m_validPix;
  }
  if (lastPicValPix > 0)
  {
    bpp = (Double)m_targetBits / (Double)lastPicValPix;
  }
#endif

  Double estLambda;
  if (eSliceType == I_SLICE)
  {
    estLambda = calculateLambdaIntra(alpha, beta, pow(m_totalCostIntra/(Double)m_numberOfPixel, BETA1), bpp);
  }
  else
  {
    estLambda = alpha * pow( bpp, beta );
  }

  printf("alpha: %lf, beta: %lf, bpp: %lf, lambda: %lf\n", alpha, beta, bpp, estLambda);

  Double lastLevelLambda = -1.0;
  Double lastPicLambda   = -1.0;
  Double lastValidLambda = -1.0;
  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == m_frameLevel )
    {
#if CTU_LEVEL_LAMBDA_CLIP || BU_LEVEL_RATE_CONTROL
      lastLevelLambda = (*it)->getPicEstLambda();
#else
      lastLevelLambda = (*it)->getPicActualLambda();
#endif
    }
    lastPicLambda     = (*it)->getPicActualLambda();

    if ( lastPicLambda > 0.0 )
    {
      lastValidLambda = lastPicLambda;
    }
  }

  if ( lastLevelLambda > 0.0 )
  {
    lastLevelLambda = Clip3( 0.1, 10000.0, lastLevelLambda );
    estLambda = Clip3( lastLevelLambda * pow( 2.0, -3.0/3.0 ), lastLevelLambda * pow( 2.0, 3.0/3.0 ), estLambda );
  }

  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( 0.1, 2000.0, lastPicLambda );
    estLambda = Clip3( lastPicLambda * pow( 2.0, -10.0/3.0 ), lastPicLambda * pow( 2.0, 10.0/3.0 ), estLambda );
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( 0.1, 2000.0, lastValidLambda );
    estLambda = Clip3( lastValidLambda * pow(2.0, -10.0/3.0), lastValidLambda * pow(2.0, 10.0/3.0), estLambda );
  }
  else
  {
    estLambda = Clip3( 0.1, 10000.0, estLambda );
  }

  if ( estLambda < 0.1 )
  {
    estLambda = 0.1;
  }

  m_estPicLambda = estLambda;

  Double totalWeight = 0.0;

#if CTU_LEVEL_BIT_ALLOCATION || VCIP_2017
  Double logTotalCTUWeight = 0.0;
  for (Int i = 0; i < m_numberOfLCU; i++ )
  {
    logTotalCTUWeight += log(m_encRCSeq->getCtuWeight(i));
  }
  Double totalCTUWeight = exp(logTotalCTUWeight/m_numberOfLCU);
#endif

#if CSVT_2018
  /************************************************************Lisx optimal bit re-allocation process for the OBA class********************************************************************************************/
  if(MyAlgorithmPara::Framelevel>0)	//judge whether is the P slices
  {
    if (MyAlgorithmPara::frameallocation > 0)	
    {
      for ( Int i=0; i<m_numberOfLCU; i++ )
      {
        Double CLCU = MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel][i];
        Double KLCU = MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][i];
        Double a=-CLCU*KLCU/pow(((Double)m_LCUs[i].m_numberOfPixel),KLCU-1.0);
        Double b=-1.0/(KLCU-1.0);
#if CTU_LEVEL_BIT_ALLOCATION
        m_LCUs[i].m_bitWeight =  pow( a/(estLambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i)),b );
#else
        m_LCUs[i].m_bitWeight =  pow( a/estLambda,b );
#endif
        if ( m_LCUs[i].m_bitWeight < 0.01 )
        {
          m_LCUs[i].m_bitWeight = 0.01;
        }
        totalWeight += m_LCUs[i].m_bitWeight;
      }  
    } 
    else
    {
      //normal case
      Double bestlambda = 0.0;
      Double Templambda = m_estPicLambda;
      Double OurAlgo=0.0;
      Double ConAlgo=0.0;
      Double TaylorE3=0.0;
      Double OurSumRate=0.0;
      Double ConSumRate=0.0;
      Int IterationNum=0;
      Int *PixelNum = NULL;
      PixelNum = new Int [m_numberOfLCU];

      for ( Int i=0; i<m_numberOfLCU; i++ )
      {
        PixelNum[i] = m_LCUs[i].m_numberOfPixel;
      }

      do{	//do the iteration 
        if (IterationNum>10)
        {
          break; //the abnormal case, break with the maximum iteration number being reached
        }

        OurAlgo=0.0;
        ConAlgo=0.0;
        TaylorE3=0.0;
        OurSumRate=0.0;
        ConSumRate=0.0;
#if CTU_LEVEL_BIT_ALLOCATION
        bestlambda = MyAlgorithmPara::SolvingCubicEqua(0,m_numberOfLCU,PixelNum,MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel],MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel],Templambda,(Double)m_bitsLeft, totalCTUWeight, m_encRCSeq->getCtuWeight());
#else
        bestlambda = MyAlgorithmPara::SolvingCubicEqua(0,m_numberOfLCU,PixelNum,MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel],MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel],Templambda,(Double)m_bitsLeft);
#endif
        Templambda = bestlambda;
        for ( Int i=0; i<m_numberOfLCU; i++ )
        {

          Double CLCU = MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel][i];
          Double KLCU = MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][i];
          Double a=-CLCU*KLCU/pow(((Double)m_LCUs[i].m_numberOfPixel),KLCU-1.0);
          Double b=-1.0/(KLCU-1.0);
#if CTU_LEVEL_BIT_ALLOCATION
          Double OurRate = pow(a / (bestlambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i)), b);
#else
          Double OurRate =  pow( a/bestlambda,b );
#endif
          TaylorE3+=OurRate;
          OurSumRate+=OurRate;
#if CTU_LEVEL_BIT_ALLOCATION
          Double ConRate = pow(a / (estLambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i)), b);
#else
          Double ConRate = pow( a/estLambda,b );
#endif
          ConSumRate+=ConRate;
        }
        IterationNum++;
      } while ((((OurAlgo - ConAlgo)>0.0001) || ((TaylorE3 - m_bitsLeft)>0.01) || ((m_bitsLeft - TaylorE3)>0.01))); //Check the convergence criterion

      // Do the optimal bit allocation with the bestlambda

      for (Int i = 0; i < m_numberOfLCU; i++)
      {

        Double CLCU = MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel][i];
        Double KLCU = MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][i];
        Double a = -CLCU*KLCU / pow(((Double)m_LCUs[i].m_numberOfPixel), KLCU - 1.0);
        Double b = -1.0 / (KLCU - 1.0);
#if CTU_LEVEL_BIT_ALLOCATION
        m_LCUs[i].m_bitWeight = pow(a / (bestlambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i)), b);
#else
        m_LCUs[i].m_bitWeight = pow(a / bestlambda, b);
#endif
        if (m_LCUs[i].m_bitWeight < 0.01)
        {
          m_LCUs[i].m_bitWeight = 0.01;
        }
        totalWeight += m_LCUs[i].m_bitWeight;
      }
    }
  }
  else
  {
    /********************************************************************************************************************************************************************************************/
    for (Int i = 0; i < m_numberOfLCU; i++)
    {
      Double alphaLCU, betaLCU;
      if (m_encRCSeq->getUseLCUSeparateModel())
      {
        alphaLCU = m_encRCSeq->getLCUPara(m_frameLevel, i).m_alpha;
        betaLCU = m_encRCSeq->getLCUPara(m_frameLevel, i).m_beta;
      }
      else
      {
        alphaLCU = m_encRCSeq->getPicPara(m_frameLevel).m_alpha;
        betaLCU = m_encRCSeq->getPicPara(m_frameLevel).m_beta;
      }
#if CTU_LEVEL_BIT_ALLOCATION
      m_LCUs[i].m_bitWeight = m_LCUs[i].m_numberOfPixel * pow(estLambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i) / alphaLCU, 1.0 / betaLCU);
#else
      m_LCUs[i].m_bitWeight = m_LCUs[i].m_numberOfPixel * pow(estLambda / alphaLCU, 1.0 / betaLCU);
#endif
      if (m_LCUs[i].m_bitWeight < 0.01)
      {
        m_LCUs[i].m_bitWeight = 0.01;
      }
      totalWeight += m_LCUs[i].m_bitWeight;
    }
  }//Lisx
  for (Int i = 0; i < m_numberOfLCU; i++)
  {
    Double BUTargetBits = m_targetBits * m_LCUs[i].m_bitWeight / totalWeight;
    m_LCUs[i].m_bitWeight = BUTargetBits;
  }
#else
#if BU_LEVEL_RATE_CONTROL
  Double logTotalBUWeight = 0.0;
  for (Int i = 0; i < m_numberOfBU; i++ )
  {
    logTotalBUWeight += log(m_encRCSeq->getBUWeight(i));
  }
  Double totalBUWeight = exp(logTotalBUWeight / m_numberOfBU);

  for ( Int i = 0; i < m_numberOfBU; i++ )
  {
    Double alphaBU = m_encRCSeq->getBUPara(m_frameLevel, i).m_alpha;
    Double betaBU = m_encRCSeq->getBUPara(m_frameLevel, i).m_beta;

    m_BUs[i].m_bitWeight = m_BUs[i].m_numberOfPixel * pow( estLambda * totalBUWeight / m_encRCSeq->getBUWeight(i)/alphaBU, 1.0/betaBU );

    if ( m_BUs[i].m_bitWeight < 0.1 )
    {
      m_BUs[i].m_bitWeight = 0.1;
    }
    totalWeight += m_BUs[i].m_bitWeight;
  }

  for ( Int i = 0; i < m_numberOfBU; i++ )
  {
    Double BUTargetBits = m_bitsLeft * m_BUs[i].m_bitWeight / totalWeight;
    m_BUs[i].m_bitWeight = BUTargetBits;
  }
#else
  // initial BU bit allocation weight
  for ( Int i=0; i<m_numberOfLCU; i++ )
  {
    Double alphaLCU, betaLCU;
    if ( m_encRCSeq->getUseLCUSeparateModel() )
    {
      alphaLCU = m_encRCSeq->getLCUPara( m_frameLevel, i ).m_alpha;
      betaLCU  = m_encRCSeq->getLCUPara( m_frameLevel, i ).m_beta;
    }
    else
    {
      alphaLCU = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
      betaLCU  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
    }

#if CTU_LEVEL_BIT_ALLOCATION
    m_LCUs[i].m_bitWeight =  m_LCUs[i].m_numberOfPixel * pow( estLambda * totalCTUWeight/m_encRCSeq->getCtuWeight(i)/alphaLCU, 1.0/betaLCU );
#else
    m_LCUs[i].m_bitWeight =  m_LCUs[i].m_numberOfPixel * pow( estLambda/alphaLCU, 1.0/betaLCU );
#endif

#if VCIP_2017
    m_LCUs[i].m_bitWeight =  m_targetBits * m_encRCSeq->getCtuWeight(i) / totalCTUWeight;
#endif

    if ( m_LCUs[i].m_bitWeight < 0.01 )
    {
      m_LCUs[i].m_bitWeight = 0.01;
    }
    totalWeight += m_LCUs[i].m_bitWeight;
  }
  for ( Int i=0; i<m_numberOfLCU; i++ )
  {
    Double BUTargetBits = m_targetBits * m_LCUs[i].m_bitWeight / totalWeight;
    m_LCUs[i].m_bitWeight = BUTargetBits;
  }
#endif
#endif

  return estLambda;
}

Int TEncRCPic::estimatePicQP( Double lambda, list<TEncRCPic*>& listPreviousPictures )
{
  Int QP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 );

  Int lastLevelQP = g_RCInvalidQPValue;
  Int lastPicQP   = g_RCInvalidQPValue;
  Int lastValidQP = g_RCInvalidQPValue;
  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == m_frameLevel )
    {
      lastLevelQP = (*it)->getPicActualQP();
    }
    lastPicQP = (*it)->getPicActualQP();
    if ( lastPicQP > g_RCInvalidQPValue )
    {
      lastValidQP = lastPicQP;
    }
  }

  if ( lastLevelQP > g_RCInvalidQPValue )
  {
    QP = Clip3( lastLevelQP - 3, lastLevelQP + 3, QP );
  }

  if( lastPicQP > g_RCInvalidQPValue )
  {
    QP = Clip3( lastPicQP - 10, lastPicQP + 10, QP );
  }
  else if( lastValidQP > g_RCInvalidQPValue )
  {
    QP = Clip3( lastValidQP - 10, lastValidQP + 10, QP );
  }

  return QP;
}

Double TEncRCPic::getLCUTargetBpp(SliceType eSliceType)
{
  Int   LCUIdx    = getLCUCoded();
  Double bpp      = -1.0;
  Int avgBits     = 0;

  if (eSliceType == I_SLICE)
  {
    Int noOfLCUsLeft = m_numberOfLCU - LCUIdx + 1;
    Int bitrateWindow = min(4,noOfLCUsLeft);
    Double MAD      = getLCU(LCUIdx).m_costIntra;

    if (m_remainingCostIntra > 0.1 )
    {
      Double weightedBitsLeft = (m_bitsLeft*bitrateWindow+(m_bitsLeft-getLCU(LCUIdx).m_targetBitsLeft)*noOfLCUsLeft)/(Double)bitrateWindow;
      avgBits = Int( MAD*weightedBitsLeft/m_remainingCostIntra );
    }
    else
    {
      avgBits = Int( m_bitsLeft / m_LCULeft );
    }
    m_remainingCostIntra -= MAD;
  }
  else
  {
#if CSVT_2018

#if CTU_LEVEL_BIT_ALLOCATION
    Double logTotalCTUWeight = 0.0;
    for (Int i = 0; i < m_numberOfLCU; i++)
    {
      logTotalCTUWeight += log(m_encRCSeq->getCtuWeight(i));
    }
    Double totalCTUWeight = exp(logTotalCTUWeight / m_numberOfLCU);
#endif

    Double totalWeight = 0;
    Int realInfluenceLCU = min( g_RCLCUSmoothWindowSize, getLCULeft() ); //g_RCLCUSmoothWindowSize, the same as the original RC scheme
    Int TargetbitsForSmoothWindow=0;
    Double bestlambda = 0.0;
    Double Templambda = m_estPicLambda;
    Double OurAlgo=0.0;
    Double ConAlgo=0.0;
    Double TaylorE3=0.0;
    Double OurSumRate=0.0;
    Double ConSumRate=0.0;
    Int IterationNum=0;
    Int *PixelNum = NULL;
    Double estLambda = m_estPicLambda;
    PixelNum = new Int [m_numberOfLCU];

    for (int i=LCUIdx;i<m_numberOfLCU;i++)
    {
      totalWeight+=m_LCUs[i].m_bitWeight;
    }

    for (int i=LCUIdx;i<(LCUIdx+realInfluenceLCU);i++)
    {
      TargetbitsForSmoothWindow+=(Int)m_LCUs[i].m_bitWeight;
    }

    TargetbitsForSmoothWindow = max(TargetbitsForSmoothWindow+m_bitsLeft-(Int)totalWeight,10); //obtain the total bit-rate for the realInfluenceLCU (=4) CTUs

    for ( Int i=LCUIdx; i<(LCUIdx+realInfluenceLCU); i++ )
    {
      PixelNum[i] = m_LCUs[i].m_numberOfPixel;
    }

    //just similar with the process at frame level, details can refer to the function TEncRCPic::estimatePicLambda
    do{
      if (IterationNum>=4)
      {
        break;
      }

      OurAlgo=0.0;
      ConAlgo=0.0;
      TaylorE3=0.0;
      OurSumRate=0.0;
      ConSumRate=0.0;
#if CTU_LEVEL_BIT_ALLOCATION
      bestlambda = MyAlgorithmPara::SolvingCubicEqua(LCUIdx,(LCUIdx+realInfluenceLCU),PixelNum,MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel],MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel],Templambda,(Double)TargetbitsForSmoothWindow, totalCTUWeight, m_encRCSeq->getCtuWeight());
#else
      bestlambda = MyAlgorithmPara::SolvingCubicEqua(LCUIdx,(LCUIdx+realInfluenceLCU),PixelNum,MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel],MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel],Templambda,(Double)TargetbitsForSmoothWindow);
#endif
      Templambda = bestlambda;
      for ( Int i=LCUIdx; i<(LCUIdx+realInfluenceLCU); i++ )
      {

        Double CLCU = MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel][i];
        Double KLCU = MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][i];
        Double a=-CLCU*KLCU/pow(((Double)m_LCUs[i].m_numberOfPixel),KLCU-1.0);
        Double b=-1.0/(KLCU-1.0);
#if CTU_LEVEL_BIT_ALLOCATION
        Double OurRate =  pow( a/(bestlambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i)),b );
#else
        Double OurRate =  pow( a/bestlambda,b );
#endif
        TaylorE3+=OurRate;
        OurSumRate+=OurRate;
#if CTU_LEVEL_BIT_ALLOCATION
        Double ConRate = pow( a/(estLambda * totalCTUWeight / m_encRCSeq->getCtuWeight(i)),b );
#else
        Double ConRate = pow( a/estLambda,b );
#endif
        ConSumRate+=ConRate;
      }
      IterationNum++;
    }
    while((((OurAlgo-ConAlgo)>0.0001)||((TaylorE3-TargetbitsForSmoothWindow)>0.01)||((TargetbitsForSmoothWindow-TaylorE3)>0.01)));

    Double CLCU = MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel][LCUIdx];
    Double KLCU = MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][LCUIdx];
    Double a=-CLCU*KLCU/pow(((Double)m_LCUs[LCUIdx].m_numberOfPixel),KLCU-1.0);
    Double b=-1.0/(KLCU-1.0);

#if CTU_LEVEL_BIT_ALLOCATION
    m_LCUs[LCUIdx].m_bitWeight =  pow( a/(bestlambda * totalCTUWeight / m_encRCSeq->getCtuWeight(LCUIdx)),b );
#else
    m_LCUs[LCUIdx].m_bitWeight =  pow( a/bestlambda,b );
#endif

    if (m_LCUs[LCUIdx].m_bitWeight < 0.01)
    {
      m_LCUs[LCUIdx].m_bitWeight = 0.01;
    }

    delete[] PixelNum;

    avgBits = (Int)(m_LCUs[LCUIdx].m_bitWeight + 0.5);
#else
#if BU_LEVEL_RATE_CONTROL
    if (LCUIdx % m_numberOfLCUInBU == 0)
    {
      // reinitialize some parameters
      m_LCULeftInBU = m_numberOfLCUInBU;

      // BU level rate control
      Double totalWeight = 0;
      Int BUIdx = getBUCoded();
      for (Int i = BUIdx; i<m_numberOfBU; i++)
      {
        totalWeight += m_BUs[i].m_bitWeight;
      }
      Int realInfluenceBU = min(g_RCBUSmoothWindowSize, getBULeft());
      Int BUTargetBits = (Int)(m_BUs[BUIdx].m_bitWeight - (totalWeight - m_bitsLeft) / realInfluenceBU + 0.5);

      if (BUTargetBits < 10)
      {
        BUTargetBits = 10;
      }
      m_BUs[BUIdx].m_targetBits = BUTargetBits;
      m_BUs[BUIdx].m_targetBitsLeft = BUTargetBits;

      Double bppBU = m_BUs[BUIdx].m_targetBits / (Double)m_BUs[BUIdx].m_numberOfPixel;

      Double alpha = m_encRCSeq->getBUPara(m_frameLevel, BUIdx).m_alpha;
      Double beta = m_encRCSeq->getBUPara(m_frameLevel, BUIdx).m_beta;

      Double lambda = alpha * pow(bppBU, beta);

      Double logTotalBUWeight = 0.0;
      for (Int i = 0; i < m_numberOfBU; i++)
      {
        logTotalBUWeight += log(m_encRCSeq->getBUWeight(i));
      }
      Double totalBUWeight = exp(logTotalBUWeight / m_numberOfBU);

      Double clipPicLambda = m_estPicLambda * totalBUWeight / m_encRCSeq->getBUWeight(BUIdx);
      lambda = Clip3(clipPicLambda  * pow(2.0, -2.0 / 3.0), clipPicLambda * pow(2.0, 2.0 / 3.0), lambda);
      m_BUs[BUIdx].m_estimateLambda = lambda;                                                                               // estimate lambda

      Int qp = Int(4.2005 * log(lambda) + 13.7122 + 0.5);
      qp = Clip3(0, MAX_QP, qp);
      m_BUs[BUIdx].m_estimateQP = qp;                                                                                       // estimate QP
    }

    Double totalWeight = 0.0;
    for (Int i = LCUIdx; i < (getBUCoded() + 1) * m_numberOfLCUInBU; i++)
    {
      Double alpha = m_encRCSeq->getLCUPara(m_frameLevel, i).m_alpha;
      Double beta = m_encRCSeq->getLCUPara(m_frameLevel, i).m_beta;

      totalWeight += m_LCUs[i].m_numberOfPixel * pow((m_BUs[getBUCoded()].m_estimateLambda) / alpha, 1.0 / beta);
    }

    Double alpha = m_encRCSeq->getLCUPara(m_frameLevel, LCUIdx).m_alpha;
    Double beta = m_encRCSeq->getLCUPara(m_frameLevel, LCUIdx).m_beta;
    Double currentWeight = m_LCUs[LCUIdx].m_numberOfPixel * pow((m_BUs[getBUCoded()].m_estimateLambda) / alpha, 1.0 / beta);

    Int realInfluenceLCU = min(g_RCLCUSmoothWindowSize, getLCUInBULeft());
    avgBits = (Int)(currentWeight - (totalWeight - m_BUs[getBUCoded()].m_targetBitsLeft) / realInfluenceLCU + 0.5);
#else
    Double totalWeight = 0;
    for (Int i = LCUIdx; i < m_numberOfLCU; i++)
    {
      totalWeight += m_LCUs[i].m_bitWeight;
    }
    Int realInfluenceLCU = min(g_RCLCUSmoothWindowSize, getLCULeft());
    avgBits = (Int)(m_LCUs[LCUIdx].m_bitWeight - (totalWeight - m_bitsLeft) / realInfluenceLCU + 0.5);
#endif
#endif
  }

  if ( avgBits < 1 )
  {
    avgBits = 1;
  }

  bpp = ( Double )avgBits/( Double )m_LCUs[ LCUIdx ].m_numberOfPixel;
  m_LCUs[ LCUIdx ].m_targetBits = avgBits;

  return bpp;
}

Double TEncRCPic::getLCUEstLambda( Double bpp )
{
  Int   LCUIdx = getLCUCoded();
  Double alpha;
  Double beta;
#if CSVT_2018
  if (( m_encRCSeq->getUseLCUSeparateModel() )&&(MyAlgorithmPara::Framelevel<=0))
  {
    alpha = m_encRCSeq->getLCUPara(m_frameLevel, LCUIdx).m_alpha;
    beta = m_encRCSeq->getLCUPara(m_frameLevel, LCUIdx).m_beta;
  }
  else if (MyAlgorithmPara::Framelevel > 0)
  {
    alpha = -MyAlgorithmPara::Cpara[MyAlgorithmPara::Framelevel][LCUIdx] * MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][LCUIdx];
    beta = MyAlgorithmPara::Kpara[MyAlgorithmPara::Framelevel][LCUIdx] - 1.0;
  }
  else
  {
    alpha = m_encRCSeq->getPicPara(m_frameLevel).m_alpha;
    beta = m_encRCSeq->getPicPara(m_frameLevel).m_beta;
  }
#else
  if ( m_encRCSeq->getUseLCUSeparateModel() )
  {
    alpha = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_alpha;
    beta  = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_beta;
  }
  else
  {
    alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
    beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
  }
#endif

  Double estLambda = alpha * pow( bpp, beta );

#if CTU_LEVEL_LAMBDA_CLIP
  Double logTotalCTUWeight = 0.0;
  for (Int i = 0; i < m_numberOfLCU; i++ )
  {
    logTotalCTUWeight += log(m_encRCSeq->getCtuWeight(i));
  }
  Double totalCTUWeight = exp(logTotalCTUWeight / m_numberOfLCU);

  Double clipPicLambda = m_estPicLambda * totalCTUWeight / m_encRCSeq->getCtuWeight( LCUIdx );
#else
#if BU_LEVEL_RATE_CONTROL
  Int BUIdx = getBUCoded();
  Double clipPicLambda = m_BUs[BUIdx].m_estimateLambda;
#else
  Double clipPicLambda = m_estPicLambda;
#endif
#endif

  //for Lambda clip, LCU level clip
  Double clipNeighbourLambda = -1.0;
  for ( Int i=LCUIdx - 1; i>=0; i-- )
  {
    if ( m_LCUs[i].m_lambda > 0 )
    {
      clipNeighbourLambda = m_LCUs[i].m_lambda;
      break;
    }
  }

#if CSVT_2018
  /*Lisx: Here needs to replace the original lines*/
  if (clipNeighbourLambda > 0.0)	//release the constraints for more optimal bit allocation
  {
    estLambda = Clip3(clipNeighbourLambda * pow(2.0, -(1.0 + MyAlgorithmPara::frameallocation) / 3.0), clipNeighbourLambda * pow(2.0, (1.0 + MyAlgorithmPara::frameallocation) / 3.0), estLambda);
  }

  if (clipPicLambda > 0.0)
  {
    estLambda = Clip3(clipPicLambda * pow(2.0, -(2.0 + MyAlgorithmPara::frameallocation) / 3.0), clipPicLambda * pow(2.0, (2.0 + MyAlgorithmPara::frameallocation) / 3.0), estLambda);
  }
  else
  {
    estLambda = Clip3(10.0, 1000.0, estLambda);
  }
  /**/
#else
  if ( clipNeighbourLambda > 0.0 )
  {
    estLambda = Clip3( clipNeighbourLambda * pow( 2.0, -1.0/3.0 ), clipNeighbourLambda * pow( 2.0, 1.0/3.0 ), estLambda );
  }

  if ( clipPicLambda > 0.0 )
  {
#if BU_LEVEL_RATE_CONTROL
    estLambda = Clip3( clipPicLambda * pow( 2.0, -2.0/3.0 ), clipPicLambda * pow( 2.0, 2.0/3.0 ), estLambda );
#else
    estLambda = Clip3( clipPicLambda * pow( 2.0, -2.0/3.0 ), clipPicLambda * pow( 2.0, 2.0/3.0 ), estLambda );
#endif
  }
  else
  {
    estLambda = Clip3( 10.0, 1000.0, estLambda );
  }
#endif

  if ( estLambda < 0.1 )
  {
    estLambda = 0.1;
  }

  return estLambda;
}

Int TEncRCPic::getLCUEstQP( Double lambda, Int clipPicQP )
{
  Int LCUIdx = getLCUCoded();
  Int estQP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 );

  //for Lambda clip, LCU level clip
  Int clipNeighbourQP = g_RCInvalidQPValue;
  for ( Int i=LCUIdx - 1; i>=0; i-- )
  {
    if ( (getLCU(i)).m_QP > g_RCInvalidQPValue )
    {
      clipNeighbourQP = getLCU(i).m_QP;
      break;
    }
  }

  if ( clipNeighbourQP > g_RCInvalidQPValue )
  {
    estQP = Clip3( clipNeighbourQP - 1, clipNeighbourQP + 1, estQP );
  }

#if BU_LEVEL_RATE_CONTROL
  Int BUIdx = getBUCoded();
  clipPicQP = m_BUs[BUIdx].m_estimateQP;
#endif

#if !CTU_LEVEL_LAMBDA_CLIP
  estQP = Clip3( clipPicQP - 1, clipPicQP + 1, estQP );
#endif

  return estQP;
}

#if ERP_RC
Void TEncRCPic::updateAfterCTU( Int LCUIdx, Int bits, Distortion distortion, Int QP, Double lambda, Bool updateLCUParameter)
#else
Void TEncRCPic::updateAfterCTU( Int LCUIdx, Int bits, Int QP, Double lambda, Bool updateLCUParameter )
#endif
{
  m_LCUs[LCUIdx].m_actualBits = bits;
  m_LCUs[LCUIdx].m_QP         = QP;
  m_LCUs[LCUIdx].m_lambda     = lambda;
#if ERP_RC
  m_LCUs[LCUIdx].m_actualMSE = (Double)distortion / (Double)m_LCUs[LCUIdx].m_numberOfPixel;
  m_LCUs[LCUIdx].m_actualSSE = m_LCUs[LCUIdx].m_actualMSE * m_LCUs[LCUIdx].m_numberOfPixel;
#endif

#if CSVT_2018
  //Lisx record the actual related parameters. Here needs to replace the original lines
  MyAlgorithmPara::LCUActualLambda[LCUIdx] = lambda;
  MyAlgorithmPara::ActualBpp[LCUIdx]=( Double )m_LCUs[LCUIdx].m_actualBits/( Double )m_LCUs[LCUIdx].m_numberOfPixel;
  MyAlgorithmPara::SkippedCURatio[LCUIdx]=	MyAlgorithmPara::SkippedCURatio[LCUIdx]/(Double)m_LCUs[LCUIdx].m_numberOfPixel;
  //end 
#endif

#if BU_LEVEL_RATE_CONTROL
  Int BUIdx = getBUCoded();
  m_BUs[BUIdx].m_actualBits += bits;
#if ERP_RC
  m_BUs[BUIdx].m_actualMSE += m_LCUs[LCUIdx].m_actualMSE;
  m_BUs[BUIdx].m_actualSSE += m_LCUs[LCUIdx].m_actualSSE;
#endif
#endif

  m_LCULeft--;
  m_bitsLeft   -= bits;
  m_pixelsLeft -= m_LCUs[LCUIdx].m_numberOfPixel;

#if BU_LEVEL_RATE_CONTROL
  m_LCULeftInBU--;
  m_BUs[BUIdx].m_targetBitsLeft -= bits;
  m_BUs[BUIdx].m_actualLambda += log(lambda);
#endif

#if BU_LEVEL_RATE_CONTROL
  if ( LCUIdx % m_numberOfLCUInBU == m_numberOfLCUInBU - 1 )
  {
    m_BULeft--;
    m_BUs[BUIdx].m_actualLambda = exp( m_BUs[BUIdx].m_actualLambda / m_numberOfLCUInBU );

    Double bpp = m_BUs[BUIdx].m_actualBits / (Double)m_BUs[BUIdx].m_numberOfPixel;
    Double MSE = m_BUs[BUIdx].m_actualMSE / (Double)m_numberOfLCUInBU;
    Double inputLambda = m_BUs[BUIdx].m_actualLambda;

    // update parameters
    TRCParameter rcPara;
    Double updatedK = bpp * inputLambda / MSE;
    Double updatedC = MSE / pow(bpp, -updatedK);
    rcPara.m_alpha = updatedC * updatedK;
    rcPara.m_beta = -updatedK - 1.0;

    if (bpp > 0 && updatedK > 0.0001)
    {
      m_encRCSeq->setBUPara(m_frameLevel, BUIdx, rcPara);
    }
    else
    {
      rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
      m_encRCSeq->setBUPara(m_frameLevel, BUIdx, rcPara);
    }
  }
#endif

  if ( !updateLCUParameter )
  {
    return;
  }

  if ( !m_encRCSeq->getUseLCUSeparateModel() )
  {
    return;
  }

  Double alpha = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_alpha;
  Double beta  = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_beta;

  Int LCUActualBits   = m_LCUs[LCUIdx].m_actualBits;
  Int LCUTotalPixels  = m_LCUs[LCUIdx].m_numberOfPixel;
  Double bpp         = ( Double )LCUActualBits/( Double )LCUTotalPixels;
  Double calLambda   = alpha * pow( bpp, beta );
  Double inputLambda = m_LCUs[LCUIdx].m_lambda;

  if( inputLambda < 0.01 || calLambda < 0.01 || bpp < 0.0001 )
  {
    alpha *= ( 1.0 - m_encRCSeq->getAlphaUpdate() / 2.0 );
    beta  *= ( 1.0 - m_encRCSeq->getBetaUpdate() / 2.0 );

    alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
    beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

    TRCParameter rcPara;
    rcPara.m_alpha = alpha;
    rcPara.m_beta  = beta;

#if ERP_RC
    if (QP == g_RCInvalidQPValue && m_encRCSeq->getAdaptiveBits() == 1)
    {
      rcPara.m_validPix = 0;
    }
    else
    {
      rcPara.m_validPix = LCUTotalPixels;
    }
#endif


#if ERP_RC
    double MSE = m_LCUs[LCUIdx].m_actualMSE;
    double updatedK = bpp * inputLambda / MSE;
    double updatedC = MSE / pow(bpp, -updatedK);
    rcPara.m_alpha = updatedC * updatedK;
    rcPara.m_beta = -updatedK - 1.0;

    if (bpp > 0 && updatedK > 0.0001)
    {
      m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
    }
    else
    {
      rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
      m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
    }
#else
    m_encRCSeq->setLCUPara( m_frameLevel, LCUIdx, rcPara );
#endif

    return;
  }

  calLambda = Clip3( inputLambda / 10.0, inputLambda * 10.0, calLambda );
  alpha += m_encRCSeq->getAlphaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * alpha;
  Double lnbpp = log( bpp );
  lnbpp = Clip3( -5.0, -0.1, lnbpp );
  beta  += m_encRCSeq->getBetaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * lnbpp;

  alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
  beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

  TRCParameter rcPara;
  rcPara.m_alpha = alpha;
  rcPara.m_beta  = beta;
#if ERP_RC
  if (QP == g_RCInvalidQPValue && m_encRCSeq->getAdaptiveBits() == 1)
  {
    rcPara.m_validPix = 0;
  }
  else
  {
    rcPara.m_validPix = LCUTotalPixels;
  }
#endif


#if ERP_RC
  double MSE = m_LCUs[LCUIdx].m_actualMSE;
  double updatedK = bpp * inputLambda / MSE;
  double updatedC = MSE / pow(bpp, -updatedK);
  rcPara.m_alpha = updatedC * updatedK;
  rcPara.m_beta = -updatedK - 1.0;

  if (bpp > 0 && updatedK > 0.0001)
  {
    m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
  }
  else
  {
    rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
    m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
  }
#else
  m_encRCSeq->setLCUPara( m_frameLevel, LCUIdx, rcPara );
#endif

}

Double TEncRCPic::calAverageQP()
{
  Int totalQPs = 0;
  Int numTotalLCUs = 0;

  Int i;
  for ( i=0; i<m_numberOfLCU; i++ )
  {
    if ( m_LCUs[i].m_QP > 0 )
    {
      totalQPs += m_LCUs[i].m_QP;
      numTotalLCUs++;
    }
  }

  Double avgQP = 0.0;
  if ( numTotalLCUs == 0 )
  {
    avgQP = g_RCInvalidQPValue;
  }
  else
  {
    avgQP = ((Double)totalQPs) / ((Double)numTotalLCUs);
  }
  return avgQP;
}

Double TEncRCPic::calAverageLambda()
{
  Double totalLambdas = 0.0;
  Int numTotalLCUs = 0;

#if ERP_RC
  Double totalSSE = 0.0;
  Int totalPixels = 0;
  m_validPixelsInPic = 0;
#endif
  Int i;
  for ( i=0; i<m_numberOfLCU; i++ )
  {
    if ( m_LCUs[i].m_lambda > 0.01 )
    {
#if ERP_RC
      if (m_LCUs[i].m_QP > 0 || m_encRCSeq->getAdaptiveBits() != 1)
      {
        m_validPixelsInPic += m_LCUs[i].m_numberOfPixel;

        totalLambdas += log(m_LCUs[i].m_lambda);
        numTotalLCUs++;

        totalSSE += m_LCUs[i].m_actualSSE;
        totalPixels += m_LCUs[i].m_numberOfPixel;
      }
#else
      totalLambdas += log( m_LCUs[i].m_lambda );
      numTotalLCUs++;
#endif
    }
  }

#if ERP_RC
  m_picMSE = totalPixels > 0 ? totalSSE / (double)totalPixels : 1.0;
#endif

  Double avgLambda;
  if( numTotalLCUs == 0 )
  {
    avgLambda = -1.0;
  }
  else
  {
    avgLambda = pow( 2.7183, totalLambdas / numTotalLCUs );
  }
  return avgLambda;
}


Void TEncRCPic::updateAfterPicture( Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType)
{
  m_picActualHeaderBits = actualHeaderBits;
  m_picActualBits       = actualTotalBits;
  if ( averageQP > 0.0 )
  {
    m_picQP             = Int( averageQP + 0.5 );
  }
  else
  {
    m_picQP             = g_RCInvalidQPValue;
  }
  m_picLambda           = averageLambda;

#if CSVT_2018
  MyAlgorithmPara::PicLambda = averageLambda; //Lisx
#endif

  printf("\nthe picture lambda is %lf, the picture QP is %d\n", m_picLambda, m_picQP);

  Double alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;

  if (eSliceType == I_SLICE)
  {
    updateAlphaBetaIntra(&alpha, &beta);
  }
  else
  {
    // update parameters
    Double picActualBits = ( Double )m_picActualBits;
#if ERP_RC
    Double picActualBpp = picActualBits / (Double)m_validPixelsInPic;
#else
    Double picActualBpp = picActualBits/(Double)m_numberOfPixel;
#endif
    Double calLambda     = alpha * pow( picActualBpp, beta );
    Double inputLambda   = m_picLambda;

    if ( inputLambda < 0.01 || calLambda < 0.01 || picActualBpp < 0.0001 )
    {
      alpha *= ( 1.0 - m_encRCSeq->getAlphaUpdate() / 2.0 );
      beta  *= ( 1.0 - m_encRCSeq->getBetaUpdate() / 2.0 );

      alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
      beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

      TRCParameter rcPara;
      rcPara.m_alpha = alpha;
      rcPara.m_beta  = beta;

#if ERP_RC
      Double avgMSE = m_picMSE;
      Double updatedK = picActualBpp * averageLambda / avgMSE;
      Double updatedC = avgMSE / pow(picActualBpp, -updatedK);

      if (m_frameLevel > 0)  //only use for level > 0
      {
        rcPara.m_alpha = updatedC * updatedK;
        rcPara.m_beta = -updatedK - 1.0;
      }

      rcPara.m_validPix = m_validPixelsInPic;

      printf("the valid pixels in picture are %d\n", m_validPixelsInPic);
      printf("the alpha is %lf, the beta is %lf\n", rcPara.m_alpha, rcPara.m_beta);

      if (m_validPixelsInPic > 0)
      {
        m_encRCSeq->setPicPara(m_frameLevel, rcPara);
      }
#else
      m_encRCSeq->setPicPara( m_frameLevel, rcPara );
#endif

      return;
    }

    calLambda = Clip3( inputLambda / 10.0, inputLambda * 10.0, calLambda );
    alpha += m_encRCSeq->getAlphaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * alpha;
    Double lnbpp = log( picActualBpp );
    lnbpp = Clip3( -5.0, -0.1, lnbpp );

    beta  += m_encRCSeq->getBetaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * lnbpp;

    alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
    beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );
  }

  TRCParameter rcPara;
  rcPara.m_alpha = alpha;
  rcPara.m_beta  = beta;

#if ERP_RC
  double picActualBpp = (double)m_picActualBits / (double)m_validPixelsInPic;

  double avgMSE = m_picMSE;
  double updatedK = picActualBpp * averageLambda / avgMSE;
  double updatedC = avgMSE / pow(picActualBpp, -updatedK);
  if (m_frameLevel > 0)  //only use for level > 0
  {
    rcPara.m_alpha = updatedC * updatedK;
    rcPara.m_beta = -updatedK - 1.0;
  }

  rcPara.m_validPix = m_validPixelsInPic;

  printf("the picture actual Bpp is %lf, the picture actual MSE is %lf, the valid pixels in picture are %d\n", picActualBpp, avgMSE, m_validPixelsInPic);
  printf("the alpha is %lf, the beta is %lf\n", rcPara.m_alpha, rcPara.m_beta);

  if (m_validPixelsInPic > 0)
  {
    m_encRCSeq->setPicPara(m_frameLevel, rcPara);
  }
#else
  m_encRCSeq->setPicPara( m_frameLevel, rcPara );
#endif

  if ( m_frameLevel == 1 )
  {
    Double currLambda = Clip3( 0.1, 10000.0, m_picLambda );
    Double updateLastLambda = g_RCWeightHistoryLambda * m_encRCSeq->getLastLambda() + g_RCWeightCurrentLambda * currLambda;
    m_encRCSeq->setLastLambda( updateLastLambda );
  }
}

Int TEncRCPic::getRefineBitsForIntra( Int orgBits )
{
  Double alpha=0.25, beta=0.5582;
  Int iIntraBits;

  if (orgBits*40 < m_numberOfPixel)
  {
    alpha=0.25;
  }
  else
  {
    alpha=0.30;
  }

  iIntraBits = (Int)(alpha* pow(m_totalCostIntra*4.0/(Double)orgBits, beta)*(Double)orgBits+0.5);

  return iIntraBits;
}

Double TEncRCPic::calculateLambdaIntra(Double alpha, Double beta, Double MADPerPixel, Double bitsPerPixel)
{
  return ( (alpha/256.0) * pow( MADPerPixel/bitsPerPixel, beta ) );
}

Void TEncRCPic::updateAlphaBetaIntra(Double *alpha, Double *beta)
{
  Double lnbpp = log(pow(m_totalCostIntra / (Double)m_numberOfPixel, BETA1));
  Double diffLambda = (*beta)*(log((Double)m_picActualBits)-log((Double)m_targetBits));

  diffLambda = Clip3(-0.125, 0.125, 0.25*diffLambda);
  *alpha    =  (*alpha) * exp(diffLambda);
  *beta     =  (*beta) + diffLambda / lnbpp;
}


Void TEncRCPic::getLCUInitTargetBits()
{
  Int iAvgBits     = 0;

  m_remainingCostIntra = m_totalCostIntra;
  for (Int i=m_numberOfLCU-1; i>=0; i--)
  {
    iAvgBits += Int(m_targetBits * getLCU(i).m_costIntra/m_totalCostIntra);
    getLCU(i).m_targetBitsLeft = iAvgBits;
  }
}


Double TEncRCPic::getLCUEstLambdaAndQP(Double bpp, Int clipPicQP, Int *estQP)
{
  Int   LCUIdx = getLCUCoded();

  Double   alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double   beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;

  Double costPerPixel = getLCU(LCUIdx).m_costIntra/(Double)getLCU(LCUIdx).m_numberOfPixel;
  costPerPixel = pow(costPerPixel, BETA1);
  Double estLambda = calculateLambdaIntra(alpha, beta, costPerPixel, bpp);

  Int clipNeighbourQP = g_RCInvalidQPValue;
  for (Int i=LCUIdx-1; i>=0; i--)
  {
    if ((getLCU(i)).m_QP > g_RCInvalidQPValue)
    {
      clipNeighbourQP = getLCU(i).m_QP;
      break;
    }
  }

  Int minQP = clipPicQP - 2;
  Int maxQP = clipPicQP + 2;

  if ( clipNeighbourQP > g_RCInvalidQPValue )
  {
    maxQP = min(clipNeighbourQP + 1, maxQP);
    minQP = max(clipNeighbourQP - 1, minQP);
  }

  Double maxLambda=exp(((Double)(maxQP+0.49)-13.7122)/4.2005);
  Double minLambda=exp(((Double)(minQP-0.49)-13.7122)/4.2005);

  estLambda = Clip3(minLambda, maxLambda, estLambda);

  *estQP = Int( 4.2005 * log(estLambda) + 13.7122 + 0.5 );
  *estQP = Clip3(minQP, maxQP, *estQP);

  return estLambda;
}

TEncRateCtrl::TEncRateCtrl()
{
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;
  m_encRCPic = NULL;
}

TEncRateCtrl::~TEncRateCtrl()
{
  destroy();
}

Void TEncRateCtrl::destroy()
{
  if ( m_encRCSeq != NULL )
  {
    delete m_encRCSeq;
    m_encRCSeq = NULL;
  }
  if ( m_encRCGOP != NULL )
  {
    delete m_encRCGOP;
    m_encRCGOP = NULL;
  }
  while ( m_listRCPictures.size() > 0 )
  {
    TEncRCPic* p = m_listRCPictures.front();
    m_listRCPictures.pop_front();
    delete p;
  }
}

#if ERP_RC
Void TEncRateCtrl::init( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int bitDepth, Int keepHierBits, Bool useLCUSeparateModel, GOPEntry  GOPList[MAX_GOP] )
#else
Void TEncRateCtrl::init( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int keepHierBits, Bool useLCUSeparateModel, GOPEntry  GOPList[MAX_GOP] )
#endif
{
  destroy();

  Bool isLowdelay = true;
  for ( Int i=0; i<GOPSize-1; i++ )
  {
    if ( GOPList[i].m_POC > GOPList[i+1].m_POC )
    {
      isLowdelay = false;
      break;
    }
  }

  Int numberOfLevel = 1;
  Int adaptiveBit = 0;
  if ( keepHierBits > 0 )
  {
    numberOfLevel = Int( log((Double)GOPSize)/log(2.0) + 0.5 ) + 1;
  }
  if ( !isLowdelay && GOPSize == 8 )
  {
    numberOfLevel = Int( log((Double)GOPSize)/log(2.0) + 0.5 ) + 1;
  }
  numberOfLevel++;    // intra picture
  numberOfLevel++;    // non-reference picture


  Int* bitsRatio;
  bitsRatio = new Int[ GOPSize ];
  for ( Int i=0; i<GOPSize; i++ )
  {
    bitsRatio[i] = 10;
    if ( !GOPList[i].m_refPic )
    {
      bitsRatio[i] = 2;
    }
  }

  if ( keepHierBits > 0 )
  {
    Double bpp = (Double)( targetBitrate / (Double)( frameRate*picWidth*picHeight ) );
    if ( GOPSize == 4 && isLowdelay )
    {
      if ( bpp > 0.2 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 6;
      }
      else if( bpp > 0.1 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 10;
      }
      else if ( bpp > 0.05 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 12;
      }
      else
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 14;
      }

      if ( keepHierBits == 2 )
      {
        adaptiveBit = 1;
      }
    }
    else if ( GOPSize == 8 && !isLowdelay )
    {
      if ( bpp > 0.2 )
      {
        bitsRatio[0] = 15;
        bitsRatio[1] = 5;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else if ( bpp > 0.1 )
      {
        bitsRatio[0] = 20;
        bitsRatio[1] = 6;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else if ( bpp > 0.05 )
      {
        bitsRatio[0] = 25;
        bitsRatio[1] = 7;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else
      {
        bitsRatio[0] = 30;
        bitsRatio[1] = 8;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }

      if ( keepHierBits == 2 )
      {
        adaptiveBit = 2;
      }
    }
#if ERP_RC
    else if (GOPSize == 16 && !isLowdelay)
    {
      if (bpp > 0.2)
      {
        bitsRatio[0] = 10;
        bitsRatio[1] = 8;
        bitsRatio[2] = 4;
        bitsRatio[3] = 2;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 2;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 4;
        bitsRatio[10] = 2;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 2;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }
      else if (bpp > 0.1)
      {
        bitsRatio[0] = 15;
        bitsRatio[1] = 9;
        bitsRatio[2] = 4;
        bitsRatio[3] = 2;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 2;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 4;
        bitsRatio[10] = 2;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 2;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }
      else if (bpp > 0.05)
      {
        bitsRatio[0] = 40;
        bitsRatio[1] = 17;
        bitsRatio[2] = 7;
        bitsRatio[3] = 2;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 2;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 7;
        bitsRatio[10] = 2;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 2;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }
      else
      {
        bitsRatio[0] = 40;
        bitsRatio[1] = 15;
        bitsRatio[2] = 6;
        bitsRatio[3] = 3;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 3;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 6;
        bitsRatio[10] = 3;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 3;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }

      if (keepHierBits == 2)
      {
        adaptiveBit = 3;
      }
    }
#endif
    else
    {
      printf( "\n hierarchical bit allocation is not support for the specified coding structure currently.\n" );
    }
  }

  Int* GOPID2Level = new Int[ GOPSize ];
  for ( Int i=0; i<GOPSize; i++ )
  {
    GOPID2Level[i] = 1;
    if ( !GOPList[i].m_refPic )
    {
      GOPID2Level[i] = 2;
    }
  }

  if ( keepHierBits > 0 )
  {
    if ( GOPSize == 4 && isLowdelay )
    {
      GOPID2Level[0] = 3;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 1;
    }
    else if ( GOPSize == 8 && !isLowdelay )
    {
      GOPID2Level[0] = 1;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 4;
      GOPID2Level[4] = 4;
      GOPID2Level[5] = 3;
      GOPID2Level[6] = 4;
      GOPID2Level[7] = 4;
    }
#if ERP_RC
    else if (GOPSize == 16 && !isLowdelay)
    {
      GOPID2Level[0] = 1;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 4;
      GOPID2Level[4] = 5;
      GOPID2Level[5] = 5;
      GOPID2Level[6] = 4;
      GOPID2Level[7] = 5;
      GOPID2Level[8] = 5;
      GOPID2Level[9] = 3;
      GOPID2Level[10] = 4;
      GOPID2Level[11] = 5;
      GOPID2Level[12] = 5;
      GOPID2Level[13] = 4;
      GOPID2Level[14] = 5;
      GOPID2Level[15] = 5;
    }
#endif
  }

  if ( !isLowdelay && GOPSize == 8 )
  {
    GOPID2Level[0] = 1;
    GOPID2Level[1] = 2;
    GOPID2Level[2] = 3;
    GOPID2Level[3] = 4;
    GOPID2Level[4] = 4;
    GOPID2Level[5] = 3;
    GOPID2Level[6] = 4;
    GOPID2Level[7] = 4;
  }
#if ERP_RC
  else if (GOPSize == 16 && !isLowdelay)
  {
    GOPID2Level[0] = 1;
    GOPID2Level[1] = 2;
    GOPID2Level[2] = 3;
    GOPID2Level[3] = 4;
    GOPID2Level[4] = 5;
    GOPID2Level[5] = 5;
    GOPID2Level[6] = 4;
    GOPID2Level[7] = 5;
    GOPID2Level[8] = 5;
    GOPID2Level[9] = 3;
    GOPID2Level[10] = 4;
    GOPID2Level[11] = 5;
    GOPID2Level[12] = 5;
    GOPID2Level[13] = 4;
    GOPID2Level[14] = 5;
    GOPID2Level[15] = 5;
  }
#endif

  m_encRCSeq = new TEncRCSeq;
  m_encRCSeq->create( totalFrames, targetBitrate, frameRate, GOPSize, picWidth, picHeight, LCUWidth, LCUHeight, numberOfLevel, useLCUSeparateModel, adaptiveBit );
  m_encRCSeq->initBitsRatio( bitsRatio );
  m_encRCSeq->initGOPID2Level( GOPID2Level );
#if ERP_RC
  m_encRCSeq->setBitDepth( bitDepth );
#endif
  m_encRCSeq->initPicPara();
#if CTU_LEVEL_BIT_ALLOCATION || VCIP_2017
  m_encRCSeq->initCtuWeight();
#endif
#if BU_LEVEL_RATE_CONTROL
  m_encRCSeq->initBUWeight();
  m_encRCSeq->initBUPara();
#endif
  if ( useLCUSeparateModel )
  {
    m_encRCSeq->initLCUPara();
  }
  m_CpbSaturationEnabled = false;
  m_cpbSize              = targetBitrate;
  m_cpbState             = (UInt)(m_cpbSize*0.5f);
  m_bufferingRate        = (Int)(targetBitrate / frameRate);

  delete[] bitsRatio;
  delete[] GOPID2Level;
}

Void TEncRateCtrl::initRCPic( Int frameLevel )
{
  m_encRCPic = new TEncRCPic;
  m_encRCPic->create( m_encRCSeq, m_encRCGOP, frameLevel, m_listRCPictures );
}

Void TEncRateCtrl::initRCGOP( Int numberOfPictures )
{
  m_encRCGOP = new TEncRCGOP;
  m_encRCGOP->create( m_encRCSeq, numberOfPictures );
}

Int  TEncRateCtrl::updateCpbState(Int actualBits)
{
  Int cpbState = 1;

  m_cpbState -= actualBits;
  if (m_cpbState < 0)
  {
    cpbState = -1;
  }

  m_cpbState += m_bufferingRate;
  if (m_cpbState > m_cpbSize)
  {
    cpbState = 0;
  }

  return cpbState;
}

Void TEncRateCtrl::initHrdParam(const TComHRD* pcHrd, Int iFrameRate, Double fInitialCpbFullness)
{
  m_CpbSaturationEnabled = true;
  m_cpbSize = (pcHrd->getCpbSizeValueMinus1(0, 0, 0) + 1) << (4 + pcHrd->getCpbSizeScale());
  m_cpbState = (UInt)(m_cpbSize*fInitialCpbFullness);
  m_bufferingRate = (UInt)(((pcHrd->getBitRateValueMinus1(0, 0, 0) + 1) << (6 + pcHrd->getBitRateScale())) / iFrameRate);
  printf("\nHRD - [Initial CPB state %6d] [CPB Size %6d] [Buffering Rate %6d]\n", m_cpbState, m_cpbSize, m_bufferingRate);
}

Void TEncRateCtrl::destroyRCGOP()
{
  delete m_encRCGOP;
  m_encRCGOP = NULL;
}
