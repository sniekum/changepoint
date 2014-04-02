/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Scott Niekum
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/**
  * \author Scott Niekum
  */

#ifndef CP_DETECTOR_H_
#define CP_DETECTOR_H_

#include "ros/ros.h"
#include "changepoint/DetectChangepoints.h"
#include <math.h>
#include <iostream>
#include <typeinfo>
#include <queue>

namespace changepoint{

class ModelParams
{
public:
    ModelParams(){};
    virtual ~ModelParams(){};
    
    virtual void printParams() = 0;
    virtual void fillParams(ModelSegment &seg) = 0;
    virtual std::string getModelName() = 0;
    // Calculates arbitrary model-specific stats for a specified data segment under the current model
    virtual std::vector< std::vector<double> > calcFinalSegStats(double **data, const int start, const int end) = 0;

    double getModelEvidence(){return modelEvidence;}
    double getLogLikelihood(){return logLikelihood;}
    
    double modelEvidence;  //If params integrated, then Bayes model evidence, otherwise probably BIC
    double logLikelihood;
};


class ModelFitter{
public:    
    ModelFitter(){};
    virtual ~ModelFitter(){};
    
    // Fit a model to the segment of start+1 to end. Params+BIC+LL in mp should be filled by fit.
    virtual bool fitSegment(double **data, const int start, const int end) = 0;
    virtual int nModels() = 0;  // Returns number of model types for this set of models

    
    ModelParams *mp;
};


class Particle
{
public:
    Particle(){};
    Particle(double prev_MAP, int pos, ModelFitter &mf){
        this->prev_MAP = prev_MAP;
        this->pos = pos;
        MAP = -INFINITY;
        nMAP = -INFINITY;
        fitter = &mf;
    }
    ~Particle(){};
    
    double MAP;
    double nMAP;
    double prev_MAP;
    int pos;
    ModelFitter *fitter;
};


class CPDetector
{
public:
    CPDetector(const std::vector<changepoint::DataPoint> data_pts);
    ~CPDetector();
    std::vector<ModelSegment> detectChangepoints();
    
private:
    int d_len, d_dim;
    double **data;
    std::vector<Particle> particles;
    std::queue<double> prev_max_MAP;
    
    double gaussCDF(double t);
    double logLenPDF(int t);
    double logOneMinLenCDF(int t);
    void normalizeMAP();
    double calculateAlpha(int M);
    void resampleParticles(int max_particles, int resamp_particles);
};

} // end namespace

#endif /* CP_DETECTOR_H_ */
