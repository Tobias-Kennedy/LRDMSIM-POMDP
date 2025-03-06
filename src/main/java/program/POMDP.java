/*******************************************************************************
 * SolvePOMDP
 * Copyright (C) 2017 Erwin Walraven
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

package program;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.lrdm.topologies.BalancedTreeTopologyStrategy;
import org.lrdm.topologies.FullyConnectedTopology;
import org.lrdm.topologies.TopologyStrategy;

import remotemirroring.RDMSimConnector;
import remotemirroring.ResultsLog;

import solver.BeliefPoint;

public class POMDP {	
	private String filename;
	private String instanceName;
	private int nStates;
	private int nActions;
	private int nObservations;
	private double discountFactor;
	private int currentState; ///Added for IoT
	
	private double[][] rewardFunction;
	//Changed transitionFunction to public from private
	public double[][][] transitionFunction;
	private double[][][] observationFunction;
	private double minReward = Double.POSITIVE_INFINITY;
	
	private BeliefPoint b0;
	
	private HashMap<Integer,String> actionLabels;
	
	public POMDP(String filename, int nStates, int nActions, int nObservations, double discountFactor, double[][] rewardFunction, double[][][] transitionFunction, double[][][] observationFunction, HashMap<Integer,String> actionLabels, BeliefPoint b0) {		
		String[] filenameSplit = filename.split("/");
		this.filename = filenameSplit[filenameSplit.length-1];
		this.instanceName = filenameSplit[filenameSplit.length-1].replace(".POMDP", "");
		this.nStates = nStates;
		this.nActions = nActions;
		this.nObservations = nObservations;
		this.discountFactor = discountFactor;
		this.rewardFunction = rewardFunction;
		this.transitionFunction = transitionFunction;
		this.observationFunction = observationFunction;
		this.actionLabels = actionLabels;
		this.b0 = b0;
		
		// compute min reward
		for(int s=0; s<nStates; s++) {
			for(int a=0; a<nActions; a++) {
				minReward = Math.min(minReward, rewardFunction[s][a]);
			}
		}
	}
	
	public int getNumStates() {
		return nStates;
	}
	
	public int getNumActions() {
		return nActions;
	}
	
	public int getNumObservations() {
		return nObservations;
	}
	
	public double getDiscountFactor() {
		return discountFactor;
	}
	
	public double getTransitionProbability(int s, int a, int sNext) {
		assert s<nStates && a<nActions && sNext<nStates;
		return transitionFunction[s][a][sNext];
	}
	
	public double getReward(int s, int a) {
		assert s<nStates && a<nActions;
		return rewardFunction[s][a];
	}
	
	public double getObservationProbability(int a, int sNext, int o) {
		assert a<nActions && sNext<nStates && o<nObservations;
		return observationFunction[a][sNext][o];
	}
	
	public double getMinReward() {
		return minReward;
	}
	
	public String getFilename() {
		return filename;
	}
	
	public String getInstanceName() {
		return instanceName;
	}
	
	public String getActionLabel(int a) {
		return actionLabels.get(a);
	}
	
	public BeliefPoint updateBelief(BeliefPoint b, int a, int o) {
		assert a<nActions && o<nObservations;
		double[] newBelief = new double[nStates];
		
		// check if belief point has been prepared
		if(!b.hasActionObservationProbabilities()) {
			prepareBelief(b);
		}
		
		// compute normalizing constant
		double nc = b.getActionObservationProbability(a, o);
		assert nc > 0.0 : "o cannot be observed when executing a in belief b";
		
		// compute the new belief vector
		for(int sNext=0; sNext<nStates; sNext++) {
			double beliefEntry = 0.0;
			
			for(int s=0; s<nStates; s++) {
				beliefEntry += getTransitionProbability(s, a, sNext) * b.getBelief(s);
			}
			
			newBelief[sNext] = beliefEntry * (getObservationProbability(a, sNext, o) / nc);
		}
		
		return new BeliefPoint(newBelief);
	}
	
	public void prepareBelief(BeliefPoint b) {
		assert b != null;
		if(b.hasActionObservationProbabilities()) return;
		
		double[][] aoProbs = new double[nActions][nObservations];
		
		for(int a=0; a<nActions; a++) {
			for(int o=0; o<nObservations; o++) {
				double prob = 0.0;
				
				for(int sNext=0; sNext<nStates; sNext++) {
					double p = 0.0;
					
					for(int s=0; s<nStates; s++) {
						p += getTransitionProbability(s, a, sNext) * b.getBelief(s);
					}
					
					prob += getObservationProbability(a, sNext, o) * p;
				}
				
				aoProbs[a][o] = prob;
			}
		}
		
		b.setActionObservationProbabilities(aoProbs);
	}
	
	public BeliefPoint getInitialBelief() {
		return b0;
	}
	/////Added for for IoT
	public void setInitialBelief(BeliefPoint b)
	{
		b0=b;
	}
	

	public int getCurrentState()
	{
		return currentState;
	}
	
	public void setCurrentState(int s)
	{
		currentState=s;
	}
	
	
	/**
	 * To perform Action for RDM
	 * @param currentState
	 * @param action
	 * @return
	 */
	public int nextStateRDM(int currentState, int selectedaction) {
		
		// apparently this if statement isn't needed?
		System.out.println("Setting network topology");
		if(selectedaction==0)
		{
			System.out.println("BT");
			RDMSimConnector.effector.setStrategy(new BalancedTreeTopologyStrategy(), RDMSimConnector.timestep);
		}
		else
		{
			System.out.println("FC");
			RDMSimConnector.effector.setStrategy(new FullyConnectedTopology(), RDMSimConnector.timestep);
		}

		
		int al=RDMSimConnector.timedRDMSim.network.getNumActiveLinks(); // used probes before
		double bw=RDMSimConnector.timedRDMSim.network.getPredictedBandwidth(RDMSimConnector.timestep);
		// double ttw=RDMSimConnector.network_management.(); // NO TTW FUNCTIONS TO USE?
		double ttw=30;
		
			
		if(bw<=40 && al>=35 && ttw<=45)
		{
			return 0;
		}
		else if(bw<=40 && al>=35 && ttw>45)
		{
			return 1;
		}
		else if(bw<=40 &&al<35 &&ttw<=45)
		{
			return 2;
		}
		else if(bw<=40 &&al<35 &&ttw>45)
		{
			return 3;
		}
		else if(bw>40 &&al>=35 &&ttw<=45)
		{
			return 4;
		}
		else if(bw>40 &&al>=35 &&ttw>45)
		{
			return 5;
		}
		else if(bw>40 &&al<35 &&ttw<=45)
		{
			return 6;
		}
		else if(bw>40 &&al<35 &&ttw>45)
		{
			return 7;
		}
		
		return 0;
	}
	
	
	public int getObservation(int action, int statePrime) {
		
		int obs=-1;

		List<Integer> bandwidthHist = new ArrayList<>(RDMSimConnector.timedRDMSim.network.getBandwidthHistory().values());
		double rec_sMEC = bandwidthHist.get(bandwidthHist.size() - 1);

		List<Integer> alHist = new ArrayList<>(RDMSimConnector.timedRDMSim.network.getActiveLinksHistory().values());
		int anl_sMR = alHist.get(alHist.size() - 1);

		List<Integer> ttwHist = new ArrayList<>(RDMSimConnector.timedRDMSim.network.getTtwHistory().values());
		double sMP = ttwHist.get(ttwHist.size() - 1);
		
		// double anl_sMR=RDMSimConnector.timedRDMSim.network.getNumLinks();
		// double rec_sMEC=RDMSimConnector.timedRDMSim.network.getPredictedBandwidth(RDMSimConnector.timestep);
		System.out.println("Number of Acrtive Links" + anl_sMR);
		// double sMP=RDMSimConnector.probe.getTimeToWrite();
		// double sMP=30;
		
		int TargetLinks = RDMSimConnector.timedRDMSim.network.getTopologyStrategy().getNumTargetLinks(RDMSimConnector.timedRDMSim.network);
		System.out.println(TargetLinks);
		
		double links_min, links_max;
		links_min=(RDMSimConnector.timedRDMSim.network.getNumActiveLinks()/3);
		links_max=(links_min*2);
		double band_min,band_max;
		band_min=(RDMSimConnector.timedRDMSim.network.getNumActiveLinks()*30/3);
		band_max=band_min*2;
		double ttw_min,ttw_max;
		ttw_min=(RDMSimConnector.timedRDMSim.network.getNumActiveLinks()*20/3);
		ttw_max=band_min*2;

		if(TargetLinks < 150){
			obs = 0;
		}
		else if(TargetLinks > 150 && TargetLinks <= 249){
			obs = 1;
		}
		else if(TargetLinks >= 250){
			obs = 2;
		}
		
	// 	if (rec_sMEC < band_min && anl_sMR < links_min & sMP<ttw_min) {
	// 		// return the observation
	// 		obs =0;

	// 	}
	// 	else if (rec_sMEC < band_min && anl_sMR < links_min&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		// return the observation
	// 		obs = 1;

	// 	}  
	// 	else if (rec_sMEC < band_min && anl_sMR < links_min&& sMP>ttw_max) {
	// 		// return the observation
	// 		obs = 2;

	// 	}  
	// 	else if (rec_sMEC < band_min && (anl_sMR >= links_min && anl_sMR <= links_max)&&sMP<ttw_min) {
	// 		obs = 3;
	// 	} 
	// 	else if (rec_sMEC < band_min && (anl_sMR >= links_min && anl_sMR <= links_max)&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		obs = 4;
	// 	}
	// 	else if (rec_sMEC < band_min && (anl_sMR >= links_min && anl_sMR <= links_max)&&sMP>ttw_max) {
	// 		obs = 5;
	// 	} 
	// 	else if (rec_sMEC < band_min && anl_sMR > links_max&&sMP<ttw_min) {
	// 		obs = 6;
	// 	} 
	// 	else if (rec_sMEC < band_min && anl_sMR > links_max&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		obs = 7;
	// 	} 
	// 	else if (rec_sMEC < band_min && anl_sMR > links_max&&sMP>ttw_max) {
	// 		obs = 8;
	// 	} 
		
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && anl_sMR < links_min&&sMP<ttw_min) {
	// 		// return the observation
	// 		obs = 9;
	// 	}
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && anl_sMR < links_min&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		// return the observation
	// 		obs = 10;
	// 	}
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && anl_sMR < links_min&&sMP>ttw_max) {
	// 		// return the observation
	// 		obs = 11;
	// 	}
		
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && (anl_sMR >= links_min && anl_sMR <= links_max)&&sMP<ttw_min) {
	// 		obs = 12;
	// 	}
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && (anl_sMR >= links_min && anl_sMR <= links_max)&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		obs = 13;
	// 	}
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && (anl_sMR >= links_min && anl_sMR <= links_max)&&sMP>ttw_max) {
	// 		obs = 14;
	// 	}
		
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && anl_sMR > links_max&&sMP<ttw_min) {
	// 		obs = 15;
	// 	}
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && anl_sMR > links_max&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		obs = 16;
	// 	}
	// 	else if ((rec_sMEC >= band_min && rec_sMEC <= band_max) && anl_sMR > links_max&&sMP>ttw_max) {
	// 		obs = 17;
	// 	}
		
		
	// 	else if ((rec_sMEC >band_max) && anl_sMR < links_min&&sMP<ttw_min) {
	// 		// return the observation
	// 		obs = 18;
	// 	} 
	// 	else if ((rec_sMEC > band_max) && anl_sMR < links_min&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		// return the observation
	// 		obs = 19;
	// 	} 
	// 	else if ((rec_sMEC >band_max) && anl_sMR < links_min&&sMP>ttw_max) {
	// 		// return the observation
	// 		obs = 20;
	// 	} 
		
		
	// 	else if ((rec_sMEC >band_max) && (anl_sMR >= links_min && anl_sMR <= links_max)&&sMP<ttw_min) {
	// 		obs = 21;
	// 	} 
	// 	else if ((rec_sMEC >band_max) && (anl_sMR >= links_min && anl_sMR <= links_max)&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		obs = 22;
	// 	} 
	// 	else if ((rec_sMEC >band_max) && (anl_sMR >= links_min && anl_sMR <= links_max)&&sMP>ttw_max) {
	// 		obs = 23;
	// 	} 
		
	// 	else if ((rec_sMEC >band_max) && anl_sMR >links_max&&sMP<ttw_min) {
	// 		obs = 24;
	// 	}
	// 	else if ((rec_sMEC >band_max) && anl_sMR > links_max&&(sMP>=ttw_min&&sMP<=ttw_max)) {
	// 		obs = 25;
	// 	}
	// 	else if ((rec_sMEC >band_max) && anl_sMR > links_max&&sMP>ttw_max) {
	// 		obs = 26;
	// 	}
		
		obs = 2;
		return obs;

	}
	
	
	public int getInitialStateRDM()
	{
		// int al=RDMSimConnector.probe.getActiveLinks();
		// double bw=RDMSimConnector.probe.getBandwidthConsumption();
		// double ttw=RDMSimConnector.probe.getTimeToWrite();

		int al=RDMSimConnector.timedRDMSim.network.getNumActiveLinks(); // used probes before
		double bw=RDMSimConnector.timedRDMSim.network.getPredictedBandwidth(RDMSimConnector.timestep);
		// double ttw=RDMSimConnector.network_management.(); // NO TTW FUNCTIONS TO USE?
		double ttw=30;
		
		ResultsLog.activelinks=al;
		ResultsLog.bandwidthconsumption=bw;
		ResultsLog.timetowrite=ttw;
		
		if(bw<=40 &&al>=35 &&ttw<=45)
		{
			ResultsLog.satmc=1;
			ResultsLog.satmr=1;
			ResultsLog.satmp=1;
			return 0;
		}
		else if(bw<=40 &&al>=35 &&ttw>45)
		{
			ResultsLog.satmc=1;
			ResultsLog.satmr=1;
			ResultsLog.satmp=0;
			return 1;
		}
		else if(bw<=40 &&al<35 &&ttw<=45)
		{
			ResultsLog.satmc=1;
			ResultsLog.satmr=0;
			ResultsLog.satmp=1;
			return 2;
		}
		else if(bw<=40 &&al<35 &&ttw>45)
		{
			ResultsLog.satmc=1;
			ResultsLog.satmr=0;
			ResultsLog.satmp=0;
			return 3;
		}
		else if(bw>40 &&al>=35 &&ttw<=45)
		{
			ResultsLog.satmc=0;
			ResultsLog.satmr=1;
			ResultsLog.satmp=1;
			return 4;
		}
		else if(bw>40 &&al>=35 &&ttw>45)
		{
			ResultsLog.satmc=0;
			ResultsLog.satmr=1;
			ResultsLog.satmp=0;
			return 5;
		}
		else if(bw>40 &&al<35 &&ttw<=45)
		{
			ResultsLog.satmc=0;
			ResultsLog.satmr=0;
			ResultsLog.satmp=1;
			return 6;
		}
		else if(bw>40 &&al<35 &&ttw>45)
		{
			ResultsLog.satmc=0;
			ResultsLog.satmr=0;
			ResultsLog.satmp=0;
			return 7;
		}
		
		
		return 0;	
	
		
	}
	

		
}
	
	

	

