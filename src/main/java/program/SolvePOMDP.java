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

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.ArrayList;
import java.util.Properties;
import java.util.Random;

import com.github.cliftonlabs.json_simple.JsonArray;
import com.github.cliftonlabs.json_simple.JsonObject;


import pruning.PruneStandard;
// import rdm.management.NetworkManagment;

import org.lrdm.TimedRDMSim;
import org.lrdm.effectors.Effector;
import org.lrdm.probes.Probe;
import org.lrdm.topologies.BalancedTreeTopologyStrategy;
import org.lrdm.topologies.FullyConnectedTopology;
import org.lrdm.topologies.NConnectedTopology;
import org.lrdm.topologies.TopologyStrategy;

// import rdm.management.RDMSimulator;
import remotemirroring.RDMSimConnector;
import remotemirroring.RDMTransitionProb;
import remotemirroring.ResultsLog;

import pruning.PruneAccelerated;
import pruning.PruneMethod;
import solver.AlphaVector;
import solver.BeliefPoint;
import solver.Solver;
import solver.SolverApproximate;
import solver.SolverExact;

import lpsolver.LPGurobi;
import lpsolver.LPModel;
import lpsolver.LPSolve;
import lpsolver.LPjoptimizer;



public class SolvePOMDP {
	private SolverProperties sp;     // object containing user-defined properties
	private PruneMethod pm;          // pruning method used by incremental pruning
	private LPModel lp;              // linear programming solver used by incremental pruning
	private Solver solver;           // the solver that we use to solve a POMDP, which is exact or approximate
	private String domainDirName;    // name of the directory containing .POMDP files
	private String domainDir;        // full path of the domain directory
	private int currentscenario_case_int;
	
	public SolvePOMDP() {
		// read parameters from config file
		readConfigFile();
		
		// check if required directories exist
		configureDirectories();

		// configure LP solver
		lp.setEpsilon(sp.getEpsilon());
		lp.setAcceleratedLPThreshold(sp.getAcceleratedLPThreshold());
		lp.setAcceleratedLPTolerance(sp.getAcceleratedLPTolerance());
		lp.setCoefficientThreshold(sp.getCoefficientThreshold());
		lp.init();
	}
	
	/**
	 * Read the solver.config file. It creates a properties object and it initializes
	 * the pruning method and LP solver.
	 */
	private void readConfigFile() {
		this.sp = new SolverProperties();
		
		Properties properties = new Properties();
		
		try {
			FileInputStream file = new FileInputStream("./solver.config");
			properties.load(file);
			file.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		sp.setEpsilon(Double.parseDouble(properties.getProperty("epsilon")));
		sp.setValueFunctionTolerance(Double.parseDouble(properties.getProperty("valueFunctionTolerance")));
		sp.setAcceleratedLPThreshold(Integer.parseInt(properties.getProperty("acceleratedLPThreshold")));
		sp.setAcceleratedLPTolerance(Double.parseDouble(properties.getProperty("acceleratedTolerance")));
		sp.setCoefficientThreshold(Double.parseDouble(properties.getProperty("coefficientThreshold")));
		sp.setOutputDirName(properties.getProperty("outputDirectory"));
		sp.setTimeLimit(Double.parseDouble(properties.getProperty("timeLimit")));
		sp.setBeliefSamplingRuns(Integer.parseInt(properties.getProperty("beliefSamplingRuns")));
		sp.setBeliefSamplingSteps(Integer.parseInt(properties.getProperty("beliefSamplingSteps")));
		this.domainDirName = properties.getProperty("domainDirectory");
		String algorithmType = properties.getProperty("algorithmType");
		
		if(!algorithmType.equals("perseus") && !algorithmType.equals("gip")) {
			throw new RuntimeException("Unexpected algorithm type in properties file");
		}
		
		String dumpPolicyGraphStr = properties.getProperty("dumpPolicyGraph");
		if(!dumpPolicyGraphStr.equals("true") && !dumpPolicyGraphStr.equals("false")) {
			throw new RuntimeException("Policy graph property must be either true or false");
		}
		else {
			sp.setDumpPolicyGraph(dumpPolicyGraphStr.equals("true") && algorithmType.equals("gip"));
		}
		
		String dumpActionLabelsStr = properties.getProperty("dumpActionLabels");
		if(!dumpActionLabelsStr.equals("true") && !dumpActionLabelsStr.equals("false")) {
			throw new RuntimeException("Action label property must be either true or false");
		}
		else {
			sp.setDumpActionLabels(dumpActionLabelsStr.equals("true"));
		}
		
		System.out.println();
		System.out.println("=== SOLVER PARAMETERS ===");
		System.out.println("Epsilon: "+sp.getEpsilon());
		System.out.println("Value function tolerance: "+sp.getValueFunctionTolerance());
		System.out.println("Accelerated LP threshold: "+sp.getAcceleratedLPThreshold());
		System.out.println("Accelerated LP tolerance: "+sp.getAcceleratedLPTolerance());
		System.out.println("LP coefficient threshold: "+sp.getCoefficientThreshold());
		System.out.println("Time limit: "+sp.getTimeLimit());
		System.out.println("Belief sampling runs: "+sp.getBeliefSamplingRuns());
		System.out.println("Belief sampling steps: "+sp.getBeliefSamplingSteps());
		System.out.println("Dump policy graph: "+sp.dumpPolicyGraph());
		System.out.println("Dump action labels: "+sp.dumpActionLabels());
		
		// load required LP solver
		String lpSolver = properties.getProperty("lpsolver");
		if(lpSolver.equals("gurobi")) {
			this.lp = new LPGurobi();
		}
		else if(lpSolver.equals("joptimizer")) {
			this.lp = new LPjoptimizer();
		}
		else if(lpSolver.equals("lpsolve")) {
			this.lp = new LPSolve();
		}
		else {
			throw new RuntimeException("Unexpected LP solver in properties file");
		}
		
		// load required pruning algorithm
		String pruningAlgorithm = properties.getProperty("pruningMethod");
		if(pruningAlgorithm.equals("standard")) {
			this.pm = new PruneStandard();
			this.pm.setLPModel(lp);
		}
		else if(pruningAlgorithm.equals("accelerated")) {
			this.pm = new PruneAccelerated();
			this.pm.setLPModel(lp);
		}
		else {
			throw new RuntimeException("Unexpected pruning method in properties file");
		}
		
		// load required POMDP algorithm
		if(algorithmType.equals("gip")) {
			this.solver = new SolverExact(sp, lp, pm);
		}
		else if(algorithmType.equals("perseus")) {
			this.solver = new SolverApproximate(sp, new Random(222));
		}
		else {
			throw new RuntimeException("Unexpected algorithm type in properties file");
		}
		
		System.out.println("Algorithm: "+algorithmType);
		System.out.println("LP solver: "+lp.getName());
	}
	
	/**
	 * Checks if the desired domain and output directories exist, and it sets the full path to these directories.
	 */
	private void configureDirectories() {
		String path = SolvePOMDP.class.getProtectionDomain().getCodeSource().getLocation().getPath();
		String decodedPath = "";
		
		try {
			decodedPath = URLDecoder.decode(path, "UTF-8");
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
		
		if(decodedPath.endsWith(".jar")) {
			// solver has been started from jar, so we assume that output exists in the same directory as the jar file			
			int endIndex = decodedPath.lastIndexOf("/");
			String workingDir = decodedPath.substring(0, endIndex);
			sp.setWorkingDir(workingDir);
			domainDir = workingDir+"/"+domainDirName;
		}
		else {
			// solver has not been started from jar, so we assume that output exists in the current directory
			sp.setWorkingDir("");
			domainDir = domainDirName;
		}	

		File dir = new File(sp.getOutputDir());
		if(!dir.exists() || !dir.isDirectory()) {
			throw new RuntimeException("Output directory could not be found");
		}
		
		dir = new File(domainDir);
		if(!dir.exists() || !dir.isDirectory()) {
			throw new RuntimeException("Domain directory could not be found");
		}
		
		System.out.println("Output directory: "+sp.getOutputDir());
		System.out.println("Domain directory: "+domainDir);
	}
	
	/**
	 * Close the LP solvers
	 */
	public void close () {
		lp.close();
	}
	
	/**
	 * Solve a POMDP defined by a .POMDP file
	 * @param pomdpFileName filename of a domain in the domain directory
	 */
	public void run(String pomdpFileName) {
		
		
		if(pomdpFileName.equals("LRDM.POMDP"))
		{
			runCaseRDM(pomdpFileName);
		}
		
		
	}
	
	/**
	 * Method to run experiments for RDM Case Study
	 * @param pomdpFileName
	 */
	public void runCaseRDM(String pomdpFileName)
	{
		
		try {

			RDMSimConnector.timedRDMSim = new TimedRDMSim();
			RDMSimConnector.timedRDMSim.initialize(new NConnectedTopology());

			//Results Regression
			FileWriter fw_mc_regr=new FileWriter("MCRegressionResultsSolvePOMDP.txt");
			PrintWriter pw_mc_regr=new PrintWriter(fw_mc_regr);
			FileWriter fw_mr_regr=new FileWriter("MRRegressionResultsSolvePOMDP.txt");
			PrintWriter pw_mr_regr=new PrintWriter(fw_mr_regr);
			FileWriter fw_mp_regr=new FileWriter("MPRegressionResultsSolvePOMDP.txt");
			PrintWriter pw_mp_regr=new PrintWriter(fw_mp_regr);
		
			// read POMDP file
		 	int bt_cnt=0,fc_cnt=0, nc_cnt=0;

	     	RDMSimConnector con = new RDMSimConnector();
	     
	     	POMDP pomdp = Parser.readPOMDP(domainDir+"/"+pomdpFileName);	
			
			RDMSimConnector.p=pomdp;

			
			
			for(RDMSimConnector.timestep=0;RDMSimConnector.timestep < 500;RDMSimConnector.timestep++)
			{
			
				RDMSimConnector.p=pomdp;

				if(RDMSimConnector.timestep % 2 == 0){
					RDMSimConnector.timedRDMSim.network.setTopologyStrategy(new BalancedTreeTopologyStrategy(), RDMSimConnector.timestep);

				}
				else{
					RDMSimConnector.timedRDMSim.network.setTopologyStrategy(new NConnectedTopology(), RDMSimConnector.timestep);

				}

				System.out.println("Timestep: " + RDMSimConnector.timestep + "No of mirrors: " + RDMSimConnector.timedRDMSim.network.getNumMirrors());
				int TargetLinks = RDMSimConnector.timedRDMSim.network.getTopologyStrategy().getNumTargetLinks(RDMSimConnector.timedRDMSim.network);
				System.out.println("Target Links:" + TargetLinks);
	
				RDMSimConnector.timedRDMSim.network.setTopologyStrategy(new BalancedTreeTopologyStrategy(), RDMSimConnector.timestep);
				System.out.println("Target Links:" + TargetLinks);
				
				// RDMSimConnector.monitorables=RDMSimConnector.network_management.getMonitorables();
				
				System.out.println("timestep: "+RDMSimConnector.timestep);
				
				int cs=pomdp.getInitialStateRDM();
				System.out.println("Initial state: "+cs);
				pomdp.setCurrentState(cs);
					
				System.out.println("current state: "+ pomdp.getCurrentState());
				
				
				BeliefPoint initialbelief=pomdp.getInitialBelief();
				double b[]=initialbelief.getBelief();
				System.out.println("Initial Belief: "+b[0]+" "+b[1]+" "+b[2]+" "+b[3]+" "+b[4]+" "+b[5]+" "+b[6]+" "+b[7]);
				double mcsatprob=b[0]+b[1]+b[2]+b[3];
				double mrsatprob=b[0]+b[1]+b[4]+b[5];
				double mpsatprob=b[0]+b[2]+b[4]+b[6];
				
				////Results Log Regression////////
				
				pw_mc_regr.println(ResultsLog.bandwidthconsumption+","+mcsatprob+","+ResultsLog.satmc);
				pw_mr_regr.println(ResultsLog.activelinks+","+mrsatprob+","+ResultsLog.satmr);
				pw_mp_regr.println(ResultsLog.timetowrite+","+mpsatprob+","+ResultsLog.satmp);
				
				
				
				
				
				////////////////////////////////
				
				
				ArrayList<AlphaVector> V1=solver.solve(pomdp);
				System.out.println("Value size: "+V1.size()+"  Action label: "+ V1.get(0).getAction());
				
				for(int i=0;i<V1.size();i++)
				{
					System.out.println("~~~~~~~~~~~~~~~~~~~~~~~");
					System.out.println("Action label: "+ V1.get(i).getAction());
					System.out.println("~~~~~~~~~~~~~~~~~~~~~~~");
					double expectedvalue=V1.get(i).getDotProduct(pomdp.getInitialBelief().getBelief());
					System.out.println("Expected Value: "+ expectedvalue);
					
				}
				
				
				
				int bestindex=AlphaVector.getBestVectorIndex(pomdp.getInitialBelief().getBelief(), V1);
				int selectedAction=V1.get(bestindex).getAction();
				System.out.println("Selected Action: "+selectedAction);
				
				if(selectedAction==0)
				{
					bt_cnt++;
				}
				else if (selectedAction==1)
				{
					fc_cnt++;
				}
				else
				{
					nc_cnt++;
				}
				// pwaction.println(timestep+" "+selectedAction);
				// pwaction.flush();
				
				//obj.put("Selected Action", selectedAction+"");
				pomdp.setInitialBelief(initialbelief);
				RDMSimConnector.p=pomdp;
				
				///Check Perform Action
				con.performAction(selectedAction);
				pomdp=RDMSimConnector.p;
				System.out.println("Current State: "+pomdp.getCurrentState());
				
				System.out.println("\nTopology Count:: BT: "+bt_cnt+" FC: "+fc_cnt+"NC: "+nc_cnt);
				
			}
			
			pw_mc_regr.flush();
			pw_mp_regr.flush();
			pw_mr_regr.flush();
			pw_mc_regr.close();
			pw_mr_regr.close();
			pw_mp_regr.close(); 
		}

		catch(IOException ioex)
		{
			ioex.printStackTrace();
		}
	     
		
	}
	
	
	/**
	 * Main entry point of the SolvePOMDP software
	 * @param args first argument should be a filename of a .POMDP file
	 */
	public static void main(String[] args) {		
		System.out.println("SolvePOMDP v0.0.3");
		System.out.println("Author: Erwin Walraven");
		System.out.println("Web: erwinwalraven.nl/solvepomdp");
		System.out.println("Delft University of Technology");
		
		if(args.length == 0) {
			System.out.println();
			System.out.println("First argument must be the name of a file in the domains directory!");
			//System.exit(0);
		}
		
		SolvePOMDP ps = new SolvePOMDP();
		
		ps.run("LRDM.POMDP");
		ps.close();
		
	}
}
