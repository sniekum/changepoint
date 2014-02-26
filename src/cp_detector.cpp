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

#include "changepoint/cp_detector.h"
#include "changepoint/articulation.h"
#include "changepoint/gauss1D.h"

#define SQRT2PI 2.50662827463
#define LEN_MEAN 100.0                // Mean of segment length gaussian 
#define LEN_SIG 1000.0                // Sigma of segment length gaussian
#define MIN_SEG_LEN 30                // The minimum length of a segment for model fitting purposes
#define MAX_PARTICLES 10              // The most particles to ever keep in the filter
#define RESAMP_PARTICLES 10           // The number of particles to resample back down to when resampling 

using namespace std;
using namespace articulation_models;
using namespace articulation_msgs;

namespace changepoint{
    
// Choose which model(s) and fitter(s) to use 
//typedef Gauss1DParams ParamsType;
//typedef Gauss1DFitter FitterType;
typedef ArticulationParams ParamsType;
typedef ArticulationFitter FitterType;

    
CPDetector::CPDetector(const vector<DataPoint> data_pts)
{
    d_len = data_pts.size();
    if(d_len > 0){
        d_dim =  data_pts[0].point.size();
    }
    
    data = new double*[d_len];
    for(int i=0; i<d_len; i++){
        data[i] = new double[d_dim];
        for(int j=0; j<d_dim; j++){
            data[i][j] = data_pts[i].point[j];
        }
    }
}

CPDetector::~CPDetector()
{
    for(int i=0; i<d_len; i++)
        delete[] data[i];
    delete[] data;     
}


// Gaussian cdf calculation
double CPDetector::gaussCDF(double t){
    return 0.5 * erfc(-(t-LEN_MEAN)/(LEN_SIG*M_SQRT2));
}


// Calc log length distribution with discretized truncated Gaussian distribution
// NOTE: This only is accurate up to a normalizing constant introduced by truncation.
double CPDetector::logLenPDF(int t){
    // Integrate between t-1 and t, so that the CDF is consistent later for cdf(t) = pdf(t) + pdf(t-1) + ... + pdf(MIN_SEG_LEN) 
    return log( gaussCDF(t) - gaussCDF(t-1.0) );     
}


// Calc cdf of log len dist by truncating normal gauss cdf: cdf(t) - cdf(a)
// NOTE: This only is accurate up to a normalizing constant introduced by truncation.
double CPDetector::logOneMinLenCDF(int t){
    // Truncate at MIN_SEG_LEN
    double p = gaussCDF(t) - gaussCDF(MIN_SEG_LEN-1); 
    return log(1-p);
}


// Helper fxn: Sorting fxn for Particles to be used by std::sort
bool MAPSortingFxn(const Particle &i, const Particle &j) { return i.MAP < j.MAP; }


// Calculate the alpha value for SOR resampling.  M is num particles to reasmple down to
double CPDetector::calculateAlpha(int M)
{   
    //for(size_t i=1; i<particles.size(); i++)
    //    printf("Part %i:  pos: %i  Norm: %f  MAP: %f\n", i, particles[i].pos, particles[i].nMAP, particles[i].MAP);
    
    // Implementation of algo from Fearnhead & Clifford (2003)
    // Look for smallest kappa = w_i that satisfies sum[min(w_i/kappa , 1)] <= M
    int N = particles.size();
    for(int i=N-M-1; i<N; i++){   // i is the cutoff index
        int A_i = N-i-1;          // Number of elements > w_i
        double B_i = 0;           // Sum of elements <= w_i
        for(int j=0; j<=i; j++)
            B_i += particles[j].nMAP;
        
        double kappa = particles[i].nMAP;
        if(kappa == 0) continue;                  // Account for underflow
        double stat = (1/kappa)*B_i + A_i;        // Check if sum[min(w_i/kappa , 1)] <= M
        if(stat <= M){
             double alpha = B_i / (M - A_i);      // Prob mass of w_i <= kappa div by size of set
             //printf("i %i  kappa %f  A_i %i  B_i %f  Stat: %f  M: %i  Alpha: %f\n", i, kappa, A_i, B_i, stat, M, alpha);
             return alpha;
        }
    }
    
    printf("ERROR: calculateAlpha(): No suitible alpha value found\n");
    return -1;
}


// Normalize the MAP values (into nMAP) of the vector of particles. These are NOT log.
void CPDetector::normalizeMAP()
{   
    // Find the max log
    double max_log = particles[0].MAP;
    for(size_t i=1; i<particles.size(); i++){
        if(particles[i].MAP > max_log)
            max_log = particles[i].MAP;
    }
    
    // Factor out (subtract) the max log and un-log them. Some may still underflow, but they are basically zero anyway.
    for(size_t i=0; i<particles.size(); i++)
        particles[i].nMAP = exp(particles[i].MAP - max_log);
    
    // Normalize
    double total = 0;
    for(size_t i=0; i<particles.size(); i++){
        double curr = particles[i].nMAP;
        if(!isnan(curr) && curr != -INFINITY)
            total += curr;
    }
    for(size_t i=0; i<particles.size(); i++){
        double norm = particles[i].nMAP / total;
        particles[i].nMAP = norm;
    }
    
}


// Resample down to resamp_particles if there are more than max_particles, using Stratified Optimal Resampling.
void CPDetector::resampleParticles(int max_particles, int resamp_particles)
{
    // Get ready to perform resampling if there are too many particles 
    if((int)particles.size() > max_particles){
        // Normalize MAP values of all particles
        normalizeMAP();
        
        // Throw out particles with a p->nMAP value of -INFINITY or NAN
        for(std::vector<Particle>::iterator iter = particles.begin(); iter != particles.end(); ) {
            if(isnan(iter->nMAP) || iter->nMAP == -INFINITY)
                iter = particles.erase(iter);
            else ++iter;
        }
    }
    
    // Only continue with resampling if the renormalizing step didn't get rid of enough
    if((int)particles.size() > max_particles){
        // Sort the particles by normalized p->MAP values, smallest to largest
        std::sort(particles.begin(), particles.end(), MAPSortingFxn);
        
        // Calculate alpha
        double alpha = calculateAlpha(resamp_particles);
        
        // Keep particles that have weight >= alpha
        int A=0;
        std::vector<Particle> new_particles;
        for(std::vector<Particle>::iterator iter = particles.begin(); iter != particles.end(); ) {
            if(iter->nMAP >= alpha){
                new_particles.push_back(*iter);
                iter = particles.erase(iter);
                A++;
            }
            else ++iter;
        }
            
        // Choose random u from uniform dist [0, alpha]
        double u = (rand() / double(RAND_MAX)) * alpha;
        
        // Resample using SOR(3)
        for(std::vector<Particle>::iterator iter = particles.begin(); iter != particles.end(); ) {
            u -= iter->nMAP;
            if(u <= 0){
                new_particles.push_back(*iter);
                iter = particles.erase(iter);
                u = u + alpha;
                A++;
            }
            else ++iter;
        }
        
        // If underflow caused too few particles to be resampled, choose randomly
        while(A < resamp_particles){
            int ind = (rand() / double(RAND_MAX)) * particles.size();
            new_particles.push_back(particles[ind]);
            particles.erase(particles.begin() + ind);
            A++;
        }
    
        // Replace old particles with new resampled set
        particles = new_particles;
    }    
}

vector<ModelSegment> CPDetector::detectChangepoints()
{   
    srand(time(0));
    ParamsType temp;
    int size_Q = temp.nModels();  // The number of models
    double max_MAP = log(1.0 / size_Q);    
    double l_prior = max_MAP;         // A uniform prior for all models
    
    //Initialize particle filter and viterbi stats
    particles.clear();
    for(int i=0; i<size_Q; i++){
        ParamsType *mp = new ParamsType();
        FitterType *mf = new FitterType(i);
        particles.push_back(*(new Particle(max_MAP,-1,*mp,*mf))); 
    }
    vector<int> max_path_indices;
    vector<ParamsType*> max_path_models;
    prev_max_MAP = queue<double>();
    
    // Process each time step from perspective of changepoints *BEFORE* time t
    // NOTE: By starting at MIN_SEG_LEN-1, we ensure the first segment will be >= MIN_SEG_LEN
    for(int t=MIN_SEG_LEN-1; t<d_len; t++)
    {   
        // Only create new particle for first time when a CP there would divide data in 2 halves, each of MIN_SEG_LEN
        if(t >= (2*MIN_SEG_LEN)-1){
            double prev = prev_max_MAP.front();
            prev_max_MAP.pop();
            //printf("Adding particles for p=%i with prev: %f\n",t-MIN_SEG_LEN,prev);
            
            // Create new particles for a changepoint at time t-MIN_SEG_LEN, one for each model in Q
            for(int i=0; i<size_Q; i++){
                ParamsType *mp = new ParamsType();
                FitterType *mf = new FitterType(i);
                Particle *new_p = new Particle(prev,t-MIN_SEG_LEN,*mp,*mf);
                particles.push_back(*new_p);
            }
        }
        
        // Compute fit probs for all particles
        for(size_t i=0; i<particles.size(); i++){
            Particle *p = &(particles[i]);
            int seg_len = t-p->pos;
                        
            // Fit the model and calc the data likelihood
            p->fitter->fitSegment(data, p->pos, t, p->fit_params);   
            
            // p_tjq is the prob of the CP **PRIOR** to time t occuring at j (p_pos)
            // p->MAP is the prob of the MAP CP occuring at time j (p_pos)
            double p_tjq;
            double p_ME = p->fit_params->getModelEvidence();
            if(isnan(p_ME) || p_ME == -INFINITY)
                p_tjq = -INFINITY;
            else
                p_tjq = logOneMinLenCDF(seg_len-1) + p_ME + l_prior + p->prev_MAP; 
            
            if(isnan(p_tjq) || p_tjq == -INFINITY)
                p->MAP = -INFINITY;
            else
                p->MAP = p_tjq + logLenPDF(seg_len) - logOneMinLenCDF(seg_len-1);
            
            //printf("t %i   pos %i   model %s   ll %f   evi %f   tjq %f   map %f\n",t,p->pos,p->fit_params->getModelName().c_str(),p->fit_params->logLikelihood,p->fit_params->modelEvidence,p_tjq,p->MAP);
        
        }
                      
        // Update global stats and viterbi path
        double max = -INFINITY;
        Particle *max_particle = NULL;
        for(size_t i=0; i<particles.size(); i++){
            if(particles[i].MAP > max){ 
                max_particle = &(particles[i]);
                max = max_particle->MAP;
            }
        }
        if(max_particle != NULL){
            max_MAP = max_particle->MAP;
            max_path_indices.push_back(max_particle->pos);
            ParamsType *copy_model = new ParamsType(max_particle->fit_params);
            max_path_models.push_back(copy_model);
            cout << "MAX " << t << "  pos: " << max_particle->pos 
                 << "  model: " << max_particle->fit_params->getModelName() 
                 << "  map: " << max_particle->MAP << "\n\n";
        }
        else{
            max_path_indices.push_back(-1);
            max_path_models.push_back(NULL);
        }
        
        // If there are more than MAX_PARTICLES, resample down to RESAMP_PARTICLES
        resampleParticles(MAX_PARTICLES, RESAMP_PARTICLES);
  
        // Keep track of MIN_SEG_LEN many of the previous max_MAP values to create particles with later.
        prev_max_MAP.push(max_MAP);
    }
    
    
    // Now max_path contains the final path, so trace it
    //TODO: Re-evaluate model parameters for each segment, this time using optimization iterations?
    //TODO: Maybe also merge neighboring segments that have same model and nearly identical parameters?
    int curr_cp = d_len - 1;
    int path_index = curr_cp - MIN_SEG_LEN + 1;  //This isn't a CP number, but an index into the max_path
    vector<ModelSegment> segments; 
    
    printf("\nFINAL CHANGEPOINTS:\n");
    while(curr_cp > -1){
        // Print CP info
        cout << "start: " << (max_path_indices[path_index]+1) <<  "   model: " << (max_path_models[path_index]->getModelName()) 
             << "   len: " << (curr_cp - (max_path_indices[path_index])) << "\n", 
        max_path_models[path_index]->printParams(); 
        
        // Add a ModelSegment for the ROS message response
        ModelSegment temp;
        temp.model_name = max_path_models[path_index]->getModelName();
        temp.first_point = max_path_indices[path_index]+1;
        temp.last_point = curr_cp;
        max_path_models[path_index]->fillParams(temp);
        segments.insert(segments.begin(), temp);
        
        // Go to previous CP in chain
        curr_cp = max_path_indices[path_index];  
        path_index = curr_cp - MIN_SEG_LEN + 1;
    }
    
    return segments;
}
     
} // end namespace
